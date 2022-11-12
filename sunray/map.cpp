// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH


#include "map.h"
#include "robot.h"
#include "config.h"
#include "StateEstimator.h"
#include <Arduino.h>
#include "helper.h"

#ifndef _SAM3XA_                 // not Arduino Due
  #define FLOAT_CALC    1    // comment out line for using integer calculations instead of float  
#endif

// we check for memory corruptions by storing one additional item in all dynamic arrays and 
// checking the value of the item during free operation
#define CHECK_CORRUPT   1
#define CHECK_ID        0x4A4A

Point *CHECK_POINT = (Point*)0x12345678;  // just some arbitray address for corruption check

unsigned long memoryCorruptions = 0;        
unsigned long memoryAllocErrors = 0;

Point nextPointState;
bool nextPointinProgress = false;

Point::Point(){
  init();
}

void Point::init(){
  px = 0;
  py = 0;
}

float Point::x(){
  return ((float)px) / 100.0;
}

float Point::y(){
  return ((float)py) / 100.0;
}


Point::Point(float ax, float ay){
  px = ax * 100;
  py = ay * 100;
}

void Point::assign(Point &fromPoint){
  px = fromPoint.px;
  py = fromPoint.py;
}

void Point::setXY(float ax, float ay){
  px = ax * 100;
  py = ay * 100;
}

long Point::crc(){
  return (px + py);  
}

bool Point::read(File &file){
  byte marker = file.read();
  if (marker != 0xAA){
    CONSOLE.println("ERROR reading point: invalid marker");
    return false;
  }
  bool res = true;
  res &= (file.read((uint8_t*)&px, sizeof(px)) != 0);
  res &= (file.read((uint8_t*)&py, sizeof(py)) != 0);
  if (!res) {
    CONSOLE.println("ERROR reading point");
  }
  return res;
}

bool Point::write(File &file){
  bool res = true;
  res &= (file.write(0xAA) != 0);
  res &= (file.write((uint8_t*)&px, sizeof(px)) != 0);
  res &= (file.write((uint8_t*)&py, sizeof(py)) != 0);
  if (!res) {
    CONSOLE.println("ERROR writing point");
  }
  return res;
}


// -----------------------------------
Polygon::Polygon(){  
  init();
}


Polygon::Polygon(short aNumPoints){ 
  init();
  alloc(aNumPoints);  
}

void Polygon::init(){  
  numPoints = 0;  
  points = NULL;
}

Polygon::~Polygon(){
  // dealloc();
}

bool Polygon::alloc(short aNumPoints){
  if (aNumPoints == numPoints) return true;
  if ((aNumPoints < 0) || (aNumPoints > 10000)) {
    CONSOLE.println("ERROR Polygon::alloc invalid number");    
    return false;
  }
  Point* newPoints = new Point[aNumPoints+CHECK_CORRUPT];    
  if (newPoints == NULL) {
    CONSOLE.println("ERROR Polygon::alloc out of memory");
    memoryAllocErrors++;
    return false;
  }
  if (points != NULL){
    memcpy(newPoints, points, sizeof(Point)* min(numPoints,aNumPoints) );        
    if (points[numPoints].px != CHECK_ID) memoryCorruptions++;
    if (points[numPoints].py != CHECK_ID) memoryCorruptions++;
    delete[] points;    
  } 
  points = newPoints;              
  numPoints = aNumPoints;
  points[numPoints].px=CHECK_ID;
  points[numPoints].py=CHECK_ID;
  return true;
}

void Polygon::dealloc(){
  if (points == NULL) return;  
  if (points[numPoints].px != CHECK_ID) memoryCorruptions++;
  if (points[numPoints].py != CHECK_ID) memoryCorruptions++;
  delete[] points;  
  points = NULL;
  numPoints = 0;  
}

void Polygon::dump(){
  for (int i=0; i < numPoints; i++){
    CONSOLE.print("(");
    CONSOLE.print(points[i].x());
    CONSOLE.print(",");
    CONSOLE.print(points[i].y());
    CONSOLE.print(")");   
    if (i < numPoints-1) CONSOLE.print(",");
  }
  CONSOLE.println();
}

long Polygon::crc(){
  long crc = 0;
  for (int i=0; i < numPoints; i++){
    crc += points[i].crc();
  }
  return crc;
}

bool Polygon::read(File &file){
  byte marker = file.read();
  if (marker != 0xBB){
    CONSOLE.println("ERROR reading polygon: invalid marker");
    return false;
  }
  short num = 0;
  file.read((uint8_t*)&num, sizeof(num));
  //CONSOLE.print("reading points:");
  //CONSOLE.println(num);
  if (!alloc(num)) return false;
  for (short i=0; i < num; i++){    
    if (!points[i].read(file)) return false;
  }
  return true;
}

bool Polygon::write(File &file){
  if (file.write(0xBB) == 0) return false;  
  if (file.write((uint8_t*)&numPoints, sizeof(numPoints)) == 0) {
    CONSOLE.println("ERROR writing polygon");
    return false; 
  }
  //CONSOLE.print("writing points:");
  //CONSOLE.println(numPoints);
  for (int i=0; i < numPoints; i++){    
    if (!points[i].write(file)) return false;
  }
  return true;  
}

void Polygon::getCenter(Point &pt){
  float minX = 9999;
  float maxX = -9999;
  float minY = 9999;
  float maxY = -9999;
  for (int i=0; i < numPoints; i++){
    minX = min(minX, points[i].x());
    maxX = max(maxX, points[i].x());
    minY = min(minY, points[i].y());
    maxY = max(maxY, points[i].y());
  }
  pt.setXY( (maxX-minX)/2, (maxY-minY)/2 ); 
}

// -----------------------------------

PolygonList::PolygonList(){
  init();
}
  
PolygonList::PolygonList(short aNumPolygons){
  init();
  alloc(aNumPolygons);  
}

void PolygonList::init(){
  numPolygons = 0;
  polygons = NULL;  
}

PolygonList::~PolygonList(){
  //dealloc();
}

bool PolygonList::alloc(short aNumPolygons){  
  if (aNumPolygons == numPolygons) return true;
  if ((aNumPolygons < 0) || (aNumPolygons > 5000)) {
    CONSOLE.println("ERROR PolygonList::alloc invalid number");    
    return false;
  }
  Polygon* newPolygons = new Polygon[aNumPolygons+CHECK_CORRUPT];  
  if (newPolygons == NULL){
    CONSOLE.println("ERROR PolygonList::alloc out of memory");
    memoryAllocErrors++;
    return false;
  }
  if (polygons != NULL){
    memcpy(newPolygons, polygons, sizeof(Polygon)* min(numPolygons, aNumPolygons));        
    if (aNumPolygons < numPolygons){
      for (int i=aNumPolygons; i < numPolygons; i++){
        //polygons[i].dealloc();        
      }  
    }
    if (polygons[numPolygons].points != CHECK_POINT) memoryCorruptions++;
    delete[] polygons;    
  } 
  polygons = newPolygons;              
  numPolygons = aNumPolygons;  
  polygons[numPolygons].points = CHECK_POINT;
  return true;
}

void PolygonList::removePolygon(short polygonID) {
    if (polygonID < 0 || polygonID >= numPolygons) {
        // Ungültige Polygon-ID, nichts zu löschen
        return;
    }

    // Speicherplatz für die Punkte des zu entfernenden Polygons freigeben
    if (polygons[polygonID].points != NULL) {
        delete[] polygons[polygonID].points;
        polygons[polygonID].points = NULL;
    }

    polygons[polygonID] = polygons[numPolygons - 1];
    polygons[numPolygons - 1] = Polygon(); // Leeres Polygon (falls Polygon-Objekt verwendet wird)

    // Die Anzahl der Polygone entsprechend reduzieren
    numPolygons--;
    polygons[numPolygons].points = CHECK_POINT;
}

void PolygonList::dealloc(){
  if (polygons == NULL) return;
  for (int i=0; i < numPolygons; i++){
    polygons[i].dealloc();        
  }  
  if (polygons[numPolygons].points != CHECK_POINT) memoryCorruptions++;
  delete[] polygons;
  polygons = NULL;
  numPolygons = 0;  
}

int PolygonList::numPoints(){
  int num = 0;
  for (int i=0; i < numPolygons; i++){
     num += polygons[i].numPoints;
  }
  return num;
}

void PolygonList::dump(){
  for (int i=0; i < numPolygons; i++){
    CONSOLE.print(i);
    CONSOLE.print(":");
    polygons[i].dump();
  }  
  CONSOLE.println();
}

long PolygonList::crc(){
  long crc = 0;
  for (int i=0; i < numPolygons; i++){
    crc += polygons[i].crc();
  }
  return crc;
}

bool PolygonList::read(File &file){
  byte marker = file.read();
  if (marker != 0xCC){
    CONSOLE.println("ERROR reading polygon list: invalid marker");
    return false;
  }
  short num = 0;
  file.read((uint8_t*)&num, sizeof(num)); 
  //CONSOLE.print("reading polygon list:");
  //CONSOLE.println(num);
  if (!alloc(num)) return false;
  for (short i=0; i < num; i++){    
    if (!polygons[i].read(file)) return false;
  }
  return true;
}

bool PolygonList::write(File &file){
  if (file.write(0xCC) == 0) {
    CONSOLE.println("ERROR writing polygon list marker");
    return false;  
  } 
  if (file.write((uint8_t*)&numPolygons, sizeof(numPolygons)) == 0) {
    CONSOLE.println("ERROR writing polygon list");
    return false; 
  }
  //CONSOLE.print("writing polygon list:");
  //CONSOLE.println(numPolygons);
  for (int i=0; i < numPolygons; i++){    
    if (!polygons[i].write(file)) return false;
  }
  return true;  
}


// -----------------------------------

Node::Node(){
  init();
}

Node::Node(Point *aPoint, Node *aParentNode){
  init();
  point = aPoint;
  parent = aParentNode;  
};

void Node::init(){
  g = 0;
  h = 0;  
  f = 0;
  opened = false;
  closed = false;
  point = NULL;
  parent = NULL;
}

void Node::dealloc(){
}


// -----------------------------------



NodeList::NodeList(){
  init();
}
  
NodeList::NodeList(short aNumNodes){
  init();
  alloc(aNumNodes);  
}

void NodeList::init(){
  numNodes = 0;
  nodes = NULL;  
}

NodeList::~NodeList(){
  //dealloc();
}

bool NodeList::alloc(short aNumNodes){  
  if (aNumNodes == numNodes) return true;
  if ((aNumNodes < 0) || (aNumNodes > 20000)) {
    CONSOLE.println("ERROR NodeList::alloc invalid number");    
    return false;
  }
  Node* newNodes = new Node[aNumNodes+CHECK_CORRUPT];  
  if (newNodes == NULL){
    CONSOLE.println("ERROR NodeList::alloc");
    memoryAllocErrors++;
    return false;
  }
  if (nodes != NULL){
    memcpy(newNodes, nodes, sizeof(Node)* min(numNodes, aNumNodes));        
    if (aNumNodes < numNodes){
      for (int i=aNumNodes; i < numNodes; i++){
        //nodes[i].dealloc();        
      }  
    }
    if (nodes[numNodes].point != CHECK_POINT) memoryCorruptions++;
    delete[] nodes;    
  } 
  nodes = newNodes;              
  numNodes = aNumNodes;  
  nodes[numNodes].point=CHECK_POINT;
  return true;
}

void NodeList::dealloc(){
  if (nodes == NULL) return;
  for (int i=0; i < numNodes; i++){
    nodes[i].dealloc();        
  }  
  if (nodes[numNodes].point != CHECK_POINT) memoryCorruptions++;
  delete[] nodes;
  nodes = NULL;
  numNodes = 0;  
}



// ---------------------------------------------------------------------

// rescale to -PI..+PI
float Map::scalePI(float v)
{
  float d = v;
  while (d < 0) d+=2*PI;
  while (d >= 2*PI) d-=2*PI;
  if (d >= PI) return (-2*PI+d);
  else if (d < -PI) return (2*PI+d);
  else return d;
}


// scale setangle, so that both PI angles have the same sign
float Map::scalePIangles(float setAngle, float currAngle){
  if ((setAngle >= PI/2) && (currAngle <= -PI/2)) return (setAngle-2*PI);
    else if ((setAngle <= -PI/2) && (currAngle >= PI/2)) return (setAngle+2*PI);
    else return setAngle;
}


// compute course (angle in rad) between two points
float Map::pointsAngle(float x1, float y1, float x2, float y2){
  float dX = x2 - x1;
  float dY = y2 - y1;
  float angle = scalePI(atan2(dY, dX));           
  return angle;
}



// computes minimum distance between x radiant (current-value) and w radiant (set-value)
float Map::distancePI(float x, float w)
{
  // cases:
  // w=330 degree, x=350 degree => -20 degree
  // w=350 degree, x=10  degree => -20 degree
  // w=10  degree, x=350 degree =>  20 degree
  // w=0   degree, x=190 degree => 170 degree
  // w=190 degree, x=0   degree => -170 degree
  float d = scalePI(w - x);
  if (d < -PI) d = d + 2*PI;
  else if (d > PI) d = d - 2*PI;
  return d; 
}

// This is the Manhattan distance
float Map::distanceManhattan(Point &pos0, Point &pos1){
  float d1 = abs (pos1.x() - pos0.x());
  float d2 = abs (pos1.y() - pos0.y());
  return d1 + d2;
}



void Map::begin(){
  memoryCorruptions = 0;
  wayMode = WAY_MOW;
  trackReverse = false;
  trackSlow = false;
  useGPSfixForPosEstimation = true;
  useGPSfloatForPosEstimation = true;
  useGPSfloatForDeltaEstimation = true;
  useGPSfixForDeltaEstimation = true;
  useIMU = true;
  mowPointsIdx = 0;
  freePointsIdx = 0;
  dockPointsIdx = 0;
  shouldDock = false; 
  shouldRetryDock = false; 
  shouldMow = false;         
  mapCRC = 0;  
  CONSOLE.print("sizeof Point=");
  CONSOLE.println(sizeof(Point));  
  load();
  dump();
}

long Map::calcMapCRC(){   
  long crc = perimeterPoints.crc() + exclusions.crc() + dockPoints.crc() + mowPoints.crc();       
  //CONSOLE.print("computed map crc=");  
  //CONSOLE.println(crc);  
  return crc;
}

void Map::dump(){ 
  CONSOLE.print("map dump - mapCRC=");
  CONSOLE.println(mapCRC);
  CONSOLE.print("points: ");
  points.dump();
  CONSOLE.print("perimeter pts: ");
  CONSOLE.println(perimeterPoints.numPoints);
  //perimeterPoints.dump();
  CONSOLE.print("exclusion pts: ");
  CONSOLE.println(exclusionPointsCount);  
  CONSOLE.print("exclusions: ");  
  CONSOLE.println(exclusions.numPolygons);  
  //exclusions.dump();  
  CONSOLE.print("dock pts: ");
  CONSOLE.println(dockPoints.numPoints);
  //dockPoints.dump();
  CONSOLE.print("mow pts: ");  
  CONSOLE.println(mowPoints.numPoints);  
  //mowPoints.dump();
  if (mowPoints.numPoints > 0){
    CONSOLE.print("first mow point:");
    CONSOLE.print(mowPoints.points[0].x());
    CONSOLE.print(",");
    CONSOLE.println(mowPoints.points[0].y());
  }
  CONSOLE.print("free pts: ");
  CONSOLE.println(freePoints.numPoints);  
  CONSOLE.print("mowPointsIdx=");
  CONSOLE.print(mowPointsIdx);
  CONSOLE.print(" dockPointsIdx=");
  CONSOLE.print(dockPointsIdx);
  CONSOLE.print(" freePointsIdx=");
  CONSOLE.print(freePointsIdx);
  CONSOLE.print(" wayMode=");
  CONSOLE.println(wayMode);
  checkMemoryErrors();
}


void Map::checkMemoryErrors(){
  if (memoryCorruptions != 0){
    CONSOLE.print("********************* ERROR: memoryCorruptions=");
    CONSOLE.println(memoryCorruptions);
    CONSOLE.print(" *********************");
  } 
  if (memoryAllocErrors != 0){
    CONSOLE.print("********************* ERROR: memoryAllocErrors=");
    CONSOLE.println(memoryAllocErrors);
    CONSOLE.print(" *********************");
  } 
}


bool Map::load(){
  bool res = true;
#if defined(ENABLE_SD_RESUME)  
  CONSOLE.print("map load... ");
  if (!SD.exists("map.bin")) {
    CONSOLE.println("no map file!");
    return false;
  }
  mapFile = SD.open("map.bin", FILE_READ);
  if (!mapFile){        
    CONSOLE.println("ERROR opening file for reading");
    return false;
  }
  uint32_t marker = 0;
  mapFile.read((uint8_t*)&marker, sizeof(marker));
  if (marker != 0x00001000){
    CONSOLE.print("ERROR: invalid marker: ");
    CONSOLE.println(marker, HEX);
    return false;
  }
  res &= (mapFile.read((uint8_t*)&mapCRC, sizeof(mapCRC)) != 0); 
  res &= (mapFile.read((uint8_t*)&exclusionPointsCount, sizeof(exclusionPointsCount)) != 0);     
  res &= perimeterPoints.read(mapFile);
  res &= exclusions.read(mapFile);    
  res &= dockPoints.read(mapFile);
  res &= mowPoints.read(mapFile);        
  
  mapFile.close();  
  long expectedCRC = calcMapCRC();
  if (mapCRC != expectedCRC){
    CONSOLE.print("ERROR: invalid map CRC:");
    CONSOLE.print(mapCRC);
    CONSOLE.print(" expected:");
    CONSOLE.println(expectedCRC);
    res = false;
  }
  if (res){
    CONSOLE.println("ok");
  } else {
    CONSOLE.println("ERROR loading map");
    clearMap(); 
  }
#endif
  return res;
}


bool Map::save(){
  bool res = true;
#if defined(ENABLE_SD_RESUME)  
  CONSOLE.print("map save... ");
  mapFile = SD.open("map.bin", FILE_CREATE); // O_WRITE | O_CREAT);
  if (!mapFile){        
    CONSOLE.println("ERROR opening file for writing");
    return false;
  }
  uint32_t marker = 0x00001000;
  res &= (mapFile.write((uint8_t*)&marker, sizeof(marker)) != 0);
  res &= (mapFile.write((uint8_t*)&mapCRC, sizeof(mapCRC)) != 0);
  res &= (mapFile.write((uint8_t*)&exclusionPointsCount, sizeof(exclusionPointsCount)) != 0);
  if (res){
    res &= perimeterPoints.write(mapFile);
    res &= exclusions.write(mapFile);    
    res &= dockPoints.write(mapFile);
    res &= mowPoints.write(mapFile);        
  }      
  if (res){
    CONSOLE.println("ok");
  } else {
    CONSOLE.println("ERROR saving map");
  }
  mapFile.flush();
  mapFile.close();
#endif
  return res;    
}



void Map::finishedUploadingMap(){
  CONSOLE.println("finishedUploadingMap");
  #ifdef DRV_SIM_ROBOT
    float x;
    float y;
    float delta;
    if (getDockingPos(x, y, delta)){
      CONSOLE.println("SIM: setting robot pos to docking pos");
      robotDriver.setSimRobotPosState(x, y, delta);
    } else {
      CONSOLE.println("SIM: error getting docking pos");
      if (perimeterPoints.numPoints > 0){
        Point pt = perimeterPoints.points[0];
        //perimeterPoints.getCenter(pt);
        robotDriver.setSimRobotPosState(pt.x(), pt.y(), 0);
      }
    }
  #endif
  mapCRC = calcMapCRC();
  dump();
  save();
}
 
   
void Map::clearMap(){
  CONSOLE.println("clearMap");
  points.dealloc();    
  perimeterPoints.dealloc();    
  exclusions.dealloc();
  dockPoints.dealloc();      
  mowPoints.dealloc();    
  freePoints.dealloc();
  obstacles.dealloc();
  pathFinderObstacles.dealloc();
  pathFinderNodes.dealloc();
}

 
// set point
bool Map::setPoint(int idx, float x, float y){  
  if ((memoryCorruptions != 0) || (memoryAllocErrors != 0)){
    CONSOLE.println("ERROR setPoint: memory errors");
    return false; 
  }  
  if (idx == 0){   
    clearMap();
  }    
  if (idx % 100 == 0){
    if (freeMemory () < 20000){
      CONSOLE.println("OUT OF MEMORY");
      return false;
    }
  }
  if (points.alloc(idx+1)){
    points.points[idx].setXY(x, y);      
    return true;
  }
  return false;
}


// set number points for point type
bool Map::setWayCount(WayType type, int count){
  if ((memoryCorruptions != 0) || (memoryAllocErrors != 0)){
    CONSOLE.println("ERROR setWayCount: memory errors");
    return false; 
  }  
  switch (type){
    case WAY_PERIMETER:            
      if (perimeterPoints.alloc(count)){
        for (int i=0; i < count; i++){
          perimeterPoints.points[i].assign( points.points[i] );
        }
      }
      break;
    case WAY_EXCLUSION:      
      exclusionPointsCount = count;                  
      break;
    case WAY_DOCK:    
      if (dockPoints.alloc(count)){
        for (int i=0; i < count; i++){
          dockPoints.points[i].assign( points.points[perimeterPoints.numPoints + exclusionPointsCount + i] );
        }
      }
      break;
    case WAY_MOW:          
      if (mowPoints.alloc(count)){
        for (int i=0; i < count; i++){
          mowPoints.points[i].assign( points.points[perimeterPoints.numPoints + exclusionPointsCount + dockPoints.numPoints + i] );
        }
        if (exclusionPointsCount == 0){
          points.dealloc();
          finishedUploadingMap();
        }
      }
      break;    
    case WAY_FREE:      
      break;
    default: 
      return false;       
  }
  mowPointsIdx = 0;
  dockPointsIdx = 0;
  freePointsIdx = 0;  
  return true;
}


// set number exclusion points for exclusion
bool Map::setExclusionLength(int idx, int len){  
  if ((memoryCorruptions != 0) || (memoryAllocErrors != 0)){
    CONSOLE.println("ERROR setExclusionLength: memory errors");
    return false; 
  }  
  /*CONSOLE.print("setExclusionLength ");
  CONSOLE.print(idx);
  CONSOLE.print("=");
  CONSOLE.println(len);*/
    
  if (!exclusions.alloc(idx+1)) return false;
  if (!exclusions.polygons[idx].alloc(len)) return false;
  
  
  int ptIdx = 0;  
  for (int i=0; i < idx; i++){    
    ptIdx += exclusions.polygons[i].numPoints;    
  }    
  for (int j=0; j < len; j++){
    exclusions.polygons[idx].points[j].assign( points.points[perimeterPoints.numPoints + ptIdx] );        
    ptIdx ++;
  }
  CONSOLE.print("ptIdx=");
  CONSOLE.print(ptIdx);
  CONSOLE.print(" exclusionPointsCount=");
  CONSOLE.print(exclusionPointsCount);
  CONSOLE.println();
  if (ptIdx == exclusionPointsCount){
    points.dealloc();
    finishedUploadingMap();
  }
  
  //CONSOLE.print("exclusion ");
  //CONSOLE.print(idx);
  //CONSOLE.print(": ");
  //CONSOLE.println(exclusionLength[idx]);   
  return true;
}


// visualize result with:  https://www.graphreader.com/plotter
void Map::generateRandomMap(){
  CONSOLE.println("Map::generateRandomMap");
  clearMap();    
  int idx = 0;
  float angle = 0;
  int steps = 30;
  for (int i=0; i < steps; i++){
    float maxd = 10;
    float d = 10 + ((float)random(maxd*10))/10.0;  // -d/2;
    float x = cos(angle) * d;
    float y = sin(angle) * d; 
    setPoint(idx, x, y);
    //CONSOLE.print(idx);
    //CONSOLE.print(",");
    CONSOLE.print(x);
    CONSOLE.print(",");    
    CONSOLE.println(y);
    angle += 2*PI / ((float)steps);
    idx++;    
  }
  setWayCount(WAY_PERIMETER, steps);
  setWayCount(WAY_EXCLUSION, 0);
  setWayCount(WAY_DOCK, 0);
  setWayCount(WAY_MOW, 0);
}


// set desired progress in mowing points list
// 1.0 = 100%
// TODO: use path finder for valid free points to target point
void Map::setMowingPointPercent(float perc){
  if (mowPoints.numPoints == 0) return;
  mowPointsIdx = (int)( ((float)mowPoints.numPoints) * perc);
  if (mowPointsIdx >= mowPoints.numPoints) {
    mowPointsIdx = mowPoints.numPoints-1;
  }
}

void Map::skipNextMowingPoint(){
  if (mowPoints.numPoints == 0) return;
  mowPointsIdx++;
  if (mowPointsIdx >= mowPoints.numPoints) {
    mowPointsIdx = mowPoints.numPoints-1;
  }  
}


void Map::repeatLastMowingPoint(){
  if (mowPoints.numPoints == 0) return;
  if (mowPointsIdx > 1) {
    mowPointsIdx--;
  }
}

bool docktargetshifted = false;
Point docktargetshiftpoint;

void Map::run(){
  switch (wayMode){
    case WAY_DOCK:      
      // last docking point
      if (dockPointsIdx+1 == dockPoints.numPoints) {
        if (!docktargetshifted) {
          docktargetshifted = true;
          int shiftoption = random(3);
          // shiftoption == 0 means keep original point
          if (shiftoption == 0) {
            docktargetshiftpoint.assign( dockPoints.points[dockPointsIdx] );
          } else {
            // XXX STEFAN hier Verschiebung einbauen
            // Berechne den Winkel zwischen den beiden Punkten
            float angle = pointsAngle(dockPoints.points[dockPointsIdx-1].x(), dockPoints.points[dockPointsIdx-1].y(),
                                      dockPoints.points[dockPointsIdx].x(), dockPoints.points[dockPointsIdx].y());
            float newAngle;

            // verschiebung des Punktes um 90 grad (PI / 2) zur Linie nach links oder rechts
            if (shiftoption == 1) {
              newAngle = scalePI(angle + PI / 2);
            } else {
              newAngle = scalePI(angle - PI / 2);
            }
            
            // Berechne die Verschiebung in x- und y-Richtung bei 10cm
            float dxShifted = 0.07 * cos(newAngle);
            float dyShifted = 0.07 * sin(newAngle);

            docktargetshiftpoint.setXY( dockPoints.points[dockPointsIdx].x() + dxShifted, dockPoints.points[dockPointsIdx].y() + dyShifted );

            CONSOLE.print("STEFAN: Last docking point. Shifted: ");
            CONSOLE.print(docktargetshiftpoint.x());
            CONSOLE.print("/");
            CONSOLE.print(docktargetshiftpoint.y());
            CONSOLE.println("");
          }
        }
        targetPoint.assign( docktargetshiftpoint );

      } else if (dockPointsIdx < dockPoints.numPoints) {
        docktargetshifted = false;
        targetPoint.assign( dockPoints.points[dockPointsIdx] );
      }
      break;
    case WAY_MOW:
      if (mowPointsIdx < mowPoints.numPoints){
        targetPoint.assign( mowPoints.points[mowPointsIdx] );
      }
      break;
    case WAY_FREE:      
      if (freePointsIdx < freePoints.numPoints){
        targetPoint.assign(freePoints.points[freePointsIdx]);
      }
      break;
  } 
  percentCompleted = (((float)mowPointsIdx) / ((float)mowPoints.numPoints) * 100.0);
}

float Map::distanceToTargetPoint(float stateX, float stateY){  
  float dX = targetPoint.x() - stateX;
  float dY = targetPoint.y() - stateY;
  float targetDist = sqrt( sq(dX) + sq(dY) );    
  return targetDist;
}

float Map::distanceToLastTargetPoint(float stateX, float stateY){  
  float dX = lastTargetPoint.x() - stateX;
  float dY = lastTargetPoint.y() - stateY;
  float targetDist = sqrt( sq(dX) + sq(dY) );    
  return targetDist;
}

// check if path from last target to target to next target is a curve
bool Map::nextPointIsStraight(){
  // XXX TODO - should support WAY_FREE as well
  if (wayMode != WAY_MOW) return false;
  if (mowPointsIdx+1 >= mowPoints.numPoints) return false;     
  Point nextPt;
  nextPt.assign(mowPoints.points[mowPointsIdx+1]);  
  float angleCurr = pointsAngle(lastTargetPoint.x(), lastTargetPoint.y(), targetPoint.x(), targetPoint.y());
  float angleNext = pointsAngle(targetPoint.x(), targetPoint.y(), nextPt.x(), nextPt.y());
  angleNext = scalePIangles(angleNext, angleCurr);                    
  float diffDelta = distancePI(angleCurr, angleNext);                 
  //CONSOLE.println(fabs(diffDelta)/PI*180.0);
  return ((fabs(diffDelta)/PI*180.0) < 20);
}


// get docking position and orientation (x,y,delta)
bool Map::getDockingPos(float &x, float &y, float &delta){
  if (dockPoints.numPoints < 2) return false;
  Point dockFinalPt;
  Point dockPrevPt;
  dockFinalPt.assign(dockPoints.points[ dockPoints.numPoints-1]);  
  dockPrevPt.assign(dockPoints.points[ dockPoints.numPoints-2]);
  x = dockFinalPt.x();
  y = dockFinalPt.y();
  delta = pointsAngle(dockPrevPt.x(), dockPrevPt.y(), dockFinalPt.x(), dockFinalPt.y());  
  return true;
}             

// mower has been docked
void Map::setIsDocked(bool flag){
  //CONSOLE.print("Map::setIsDocked ");
  //CONSOLE.println(flag);
  if (flag){
    if (dockPoints.numPoints < 2) return; // keep current wayMode (not enough docking points for docking wayMode)  
    wayMode = WAY_DOCK;
    dockPointsIdx = dockPoints.numPoints-2;
    //targetPointIdx = dockStartIdx + dockPointsIdx;                     
    trackReverse = true;             
    trackSlow = true;
    useGPSfixForPosEstimation = !DOCK_IGNORE_GPS;
    useGPSfixForDeltaEstimation = !DOCK_IGNORE_GPS;    
    useGPSfloatForPosEstimation = false;  
    useGPSfloatForDeltaEstimation = false;
    useIMU = true; // false
  } else {
    wayMode = WAY_FREE;
    dockPointsIdx = 0;    
    trackReverse = false;             
    trackSlow = false;
    useGPSfixForPosEstimation = true;
    useGPSfixForDeltaEstimation = true;
    useGPSfloatForPosEstimation = true;    
    useGPSfloatForDeltaEstimation = true;
    useIMU = true;
  }  
}

bool Map::isUndocking(){
  return ((maps.wayMode == WAY_DOCK) && (maps.shouldMow));
}

bool Map::isDocking(){
  return ((maps.wayMode == WAY_DOCK) && (maps.shouldDock));
}

bool Map::retryDocking(float stateX, float stateY){
  CONSOLE.println("Map::retryDocking");    
  if (!shouldDock) {
    CONSOLE.println("ERROR retryDocking: not docking!");
    return false;  
  }  
  if (shouldRetryDock) {
    CONSOLE.println("ERROR retryDocking: already retrying!");   
    return false;
  } 
  if (dockPointsIdx > 0) dockPointsIdx--;    
  shouldRetryDock = true;
  trackReverse = true;
  return true;
}


bool Map::startDocking(float stateX, float stateY){  
  CONSOLE.println("Map::startDocking");
  if ((memoryCorruptions != 0) || (memoryAllocErrors != 0)){
    CONSOLE.println("ERROR startDocking: memory errors");
    return false; 
  }  
  shouldDock = true;
  shouldRetryDock = false;
  shouldMow = false;    
  if (dockPoints.numPoints > 0){
    if (wayMode == WAY_DOCK) {
      CONSOLE.println("skipping path planning to first docking point: already docking");    
      return true;
    }
    // find valid path from robot to first docking point      
    //freePoints.alloc(0);
    Point src;
    Point dst;
    src.setXY(stateX, stateY);    
    dst.assign(dockPoints.points[0]);        
    //findPathFinderSafeStartPoint(src, dst);      
    wayMode = WAY_FREE;              
    if (findPath(src, dst)){      
      return true;
    } else {
      CONSOLE.println("ERROR: no path");
      return false;
    }
  } else {
    CONSOLE.println("ERROR: no points");
    return false; 
  }
}

bool Map::startMowing(float stateX, float stateY){ 
  // here we also go / went through if an obstacle get added
  CONSOLE.println("Map::startMowing");
  //stressTest();
  //testIntegerCalcs();
  //return false;
  // ------
  if ((memoryCorruptions != 0) || (memoryAllocErrors != 0)){
    CONSOLE.println("ERROR startMowing: memory errors");
    return false; 
  }  
  shouldDock = false;
  shouldRetryDock = false;
  shouldMow = true;    
  // nextPointinProgress = false;
  if (mowPoints.numPoints > 0){
    // we enter also here after an obstacle
    // we need a complete recalc of situation
    if (wayMode != WAY_DOCK) {
      wayMode = WAY_MOW;
    }
    unsigned int r = nextPoint(false, stateX, stateY, false);
    if (r == 0) {
      CONSOLE.println("ERROR: no path");
      return false;
    }
    return true;
  } else {
    CONSOLE.println("ERROR: no points");
    return false; 
  }
}


void Map::clearObstacles(){  
  CONSOLE.println("clearObstacles");
  obstacles.dealloc();  
}


// add dynamic octagon obstacle in front of robot on line going from robot to target point
bool Map::addObstacle(float stateX, float stateY, float stateDelta, MotType motion){
  float r = OBSTACLE_DIAMETER / 2.0; // radius
  float move_angle;

  switch (motion){
    case MOT_BACKWARD:
      move_angle = 180;
      break;
    case MOT_RIGHT:
      move_angle = -40; // do not use 90 degress only fron wheels turn / move
      r = r * 2/3;
      break;
    case MOT_LEFT:
      move_angle = 40; // do not use 90 degress only fron wheels turn / move
      r = r * 2/3;
      break;
    case MOT_FORWARD:
      move_angle = 0;
      break;
  }

  float circle_rot = scalePI( scalePI(stateDelta) + scalePI(deg2rad(move_angle)) );

  // move center of octagon in the right position of mower
  float center_x = stateX + cos( circle_rot ) * ( r - 0.1 );
  float center_y = stateY + sin( circle_rot ) * ( r - 0.1 );

  // move center of octagon in the right position of mower
  float center_2x = stateX + cos( circle_rot ) * ( ( r - 0.1 ) * 2 );
  float center_2y = stateY + sin( circle_rot ) * ( ( r - 0.1 ) * 2 );

  CONSOLE.print("addObstacle: state: ");
  CONSOLE.print(stateX);
  CONSOLE.print("/");
  CONSOLE.print(stateY);
  CONSOLE.print(" stateDelta: ");
  CONSOLE.print(stateDelta);
  CONSOLE.print(" move_angle: ");
  CONSOLE.print(move_angle);
  CONSOLE.print(" Center: ");
  CONSOLE.print(center_x);
  CONSOLE.print("/");
  CONSOLE.println(center_y);
  // if (obstacles.numPolygons > 50){
  //   CONSOLE.println("error: too many obstacles");
  //   return false;
  // }
  int idx = obstacles.numPolygons;
  if (!obstacles.alloc(idx+1)) return false;
  if (!obstacles.polygons[idx].alloc(12)) return false;
  
  int ci = 0;
  // create circle / octagon around center angle 0 - "360"
  // obstacles.polygons[idx].points[0].setXY(center_x + cos(scalePI( deg2rad(0)   + circle_rot ) ) * r, center_y + sin(scalePI( deg2rad(0)   + circle_rot ) ) * r);
  
  // starting point in front of mower
  obstacles.polygons[idx].points[ci].setXY(center_x + cos(scalePI( deg2rad(160) + circle_rot ) ) * r, center_y + sin(scalePI( deg2rad(160) + circle_rot ) ) * r);
  ci += 1;

  obstacles.polygons[idx].points[ci].setXY(center_x + cos(scalePI( deg2rad(180) + circle_rot ) ) * r, center_y + sin(scalePI( deg2rad(160) + circle_rot ) ) * r);
  ci += 1;

  obstacles.polygons[idx].points[ci].setXY(center_x + cos(scalePI( deg2rad(200) + circle_rot ) ) * r, center_y + sin(scalePI( deg2rad(200) + circle_rot ) ) * r);
  ci += 1;
  
  obstacles.polygons[idx].points[ci].setXY(center_x + cos(scalePI( deg2rad(225) + circle_rot ) ) * r, center_y + sin(scalePI( deg2rad(225) + circle_rot ) ) * r);
  ci += 1;

  obstacles.polygons[idx].points[ci].setXY(center_x + cos(scalePI( deg2rad(270) + circle_rot ) ) * r, center_y + sin(scalePI( deg2rad(270) + circle_rot ) ) * r);
  ci += 1;
  // obstacles.polygons[idx].points[ci].setXY(center_x + cos(scalePI( deg2rad(315) + circle_rot ) ) * r, center_y + sin(scalePI( deg2rad(315) + circle_rot ) ) * r);
  // ci += 1;

  // obstacles.polygons[idx].points[ci].setXY(center_2x + cos(scalePI( deg2rad(225) + circle_rot ) ) * r, center_2y + sin(scalePI( deg2rad(225) + circle_rot ) ) * r);
  // ci += 1;

  obstacles.polygons[idx].points[ci].setXY(center_2x + cos(scalePI( deg2rad(270) + circle_rot ) ) * r, center_2y + sin(scalePI( deg2rad(270) + circle_rot ) ) * r);
  ci += 1;
  obstacles.polygons[idx].points[ci].setXY(center_2x + cos(scalePI( deg2rad(315) + circle_rot ) ) * r, center_2y + sin(scalePI( deg2rad(315) + circle_rot ) ) * r);
  ci += 1;
  
  obstacles.polygons[idx].points[ci].setXY(center_2x + cos(scalePI( deg2rad(0)   + circle_rot ) ) * r, center_2y + sin(scalePI( deg2rad(0)   + circle_rot ) ) * r);
  ci += 1;

  obstacles.polygons[idx].points[ci].setXY(center_2x + cos(scalePI( deg2rad(45)  + circle_rot ) ) * r, center_2y + sin(scalePI( deg2rad(45)  + circle_rot ) ) * r);
  ci += 1;
  obstacles.polygons[idx].points[ci].setXY(center_2x + cos(scalePI( deg2rad(90)  + circle_rot ) ) * r, center_2y + sin(scalePI( deg2rad(90)  + circle_rot ) ) * r);
  ci += 1;

  // obstacles.polygons[idx].points[ci].setXY(center_2x + cos(scalePI( deg2rad(135) + circle_rot ) ) * r, center_2y + sin(scalePI( deg2rad(135) + circle_rot ) ) * r);
  // ci += 1;

  // obstacles.polygons[idx].points[ci].setXY(center_x + cos(scalePI( deg2rad(45)  + circle_rot ) ) * r, center_y + sin(scalePI( deg2rad(45)  + circle_rot ) ) * r);
  // ci += 1;
  obstacles.polygons[idx].points[ci].setXY(center_x + cos(scalePI( deg2rad(90)  + circle_rot ) ) * r, center_y + sin(scalePI( deg2rad(90)  + circle_rot ) ) * r);
  ci += 1;

  obstacles.polygons[idx].points[ci].setXY(center_x + cos(scalePI( deg2rad(135) + circle_rot ) ) * r, center_y + sin(scalePI( deg2rad(135) + circle_rot ) ) * r);
  ci += 1;

  return true;
}

// check if given point is inside perimeter (and outside exclusions) of current map 
bool Map::isInsidePerimeterOutsideExclusions(Point &pt){
  if (!maps.pointIsInsidePolygon( maps.perimeterPoints, pt)) return false;    

  for (int idx=0; idx < maps.obstacles.numPolygons; idx++){
    if (!maps.pointIsInsidePolygon( maps.obstacles.polygons[idx], pt)) return false;
  }

  for (int idx=0; idx < maps.exclusions.numPolygons; idx++){
    if (maps.pointIsInsidePolygon( maps.exclusions.polygons[idx], pt)) return false;
  }    
  return true;
}


int Map::isPointInsideObstacle(Point pt, int skipidx){  
  for (int obst_ins=0; obst_ins < obstacles.numPolygons; obst_ins++){
    if (skipidx != obst_ins && pointIsInsidePolygon( obstacles.polygons[obst_ins], pt)){
      CONSOLE.print("point conflicts with idx: ");
      CONSOLE.print(obst_ins);
      CONSOLE.print(": ");
      CONSOLE.print(pt.x());
      CONSOLE.print(" / ");
      CONSOLE.println(pt.y());
      return obst_ins;
    }
  }
  return -1;
}

bool Map::findObstacleSafeMowPoint(Point &newTargetPoint, float stateX, float stateY){  
  Point dst;
  Point src;
  Point state;
  state.setXY(stateX, stateY);

  // mowline src to dst
  if (mowPointsIdx == 0) {
    src.assign(state);
  } else {
    src.assign(mowPoints.points[mowPointsIdx-1]);
  }

  dst.assign(mowPoints.points[mowPointsIdx]);

  // distance from position to dst
  float dist_src_to_state = distance(src, state);
  float dist_state_to_dst = distance(state, dst);

  CONSOLE.print("findObstacleSafeMowPoint:");
  CONSOLE.print(" idx: ");
  CONSOLE.print(mowPointsIdx);
  CONSOLE.print(" state: ");
  CONSOLE.print(stateX);
  CONSOLE.print(",");
  CONSOLE.print(stateY);

  CONSOLE.print(" mowPoints dst: ");
  CONSOLE.print(dst.x());
  CONSOLE.print(",");
  CONSOLE.print(dst.y());

  CONSOLE.print(" dist src: ");
  CONSOLE.print(dist_src_to_state);

  CONSOLE.print(" / dst: ");
  CONSOLE.println(dist_state_to_dst);

  float distToPath = distanceLine(stateX, stateY, src.x(), src.y(), dst.x(), dst.y());

  // get first obstacle in front of state_pos
  Point bestsec;
  float best_dist = 99999;
  for (int idx=0; idx < obstacles.numPolygons; idx++) {

    // check if obstacle intersect with mowline - check sect "front"
    Point sect;
    if (linePolygonIntersectPoint( src, dst, obstacles.polygons[idx], sect)) {
      float dist_obst = distance(src, sect);
      bool safe = (isPointInsideObstacle(sect, idx) == -1);

      CONSOLE.print("findObstacleSafeMowPoint: ");
      CONSOLE.print(idx);
      CONSOLE.print(" found obstacle front - sect: ");
      CONSOLE.print(sect.x());
      CONSOLE.print(",");
      CONSOLE.print(sect.y());
      CONSOLE.print(" safe: ");
      CONSOLE.print(safe);
      CONSOLE.print(" dist src: ");
      CONSOLE.println(dist_obst);

      // distance of sect point has to be > distance than state_pos to dst / target
      if (
          (
	    ((dist_obst - dist_src_to_state) >= TARGET_REACHED_TOLERANCE) || 
            (distToPath >= TARGET_REACHED_TOLERANCE && (fabs(dist_obst - dist_src_to_state) < 1))
	  ) && dist_obst < best_dist && safe) {
	bestsec.assign(sect);
	best_dist = dist_obst;
      } else {
        // backside of obstacle
	// dist_obst is not > dist_src_to_state - check other side of obstacle - because obstacle is on mowline
        Point sect_back;
        if (linePolygonIntersectPoint( dst, src, obstacles.polygons[idx], sect_back)) {
          float dist_obst = distance(src, sect_back);
          bool safe = (isPointInsideObstacle(sect_back, idx) == -1);

          CONSOLE.print("findObstacleSafeMowPoint:");
          CONSOLE.print(idx);
          CONSOLE.print(" found obstacle back - sect: ");
          CONSOLE.print(sect_back.x());
          CONSOLE.print(",");
          CONSOLE.print(sect_back.y());
          CONSOLE.print(" safe: ");
          CONSOLE.print(safe);
          CONSOLE.print(" dist src: ");
          CONSOLE.println(dist_obst);

          if (
              (
    	        ((dist_obst - dist_src_to_state) >= TARGET_REACHED_TOLERANCE) || 
                (distToPath >= TARGET_REACHED_TOLERANCE && (fabs(dist_obst - dist_src_to_state) < 1))
    	      ) && dist_obst < best_dist && safe) {
            bestsec.assign(sect_back);
            best_dist = dist_obst;
          }
	}
      }
    }
  }

  // no obstacle on mowline between state and dst
  if (best_dist == 99999) {
    bool safe = (isPointInsideObstacle(dst, -1) == -1);
    if (!safe) {
      CONSOLE.println("findObstacleSafeMowPoint: no further obstacle on mowline but mowpoint is inside obstacle. Skip to next real mowpoint.");
      // we need to skip to next mow point
      // try next mowing point
      if (!nextMowPoint(false, true)){
        CONSOLE.println("findObstacleSafeMowPoint error: no more mowing points reachable due to obstacles");
        return false;
      }
      return findObstacleSafeMowPoint(newTargetPoint, stateX, stateY);
    }

    CONSOLE.print("findObstacleSafeMowPoint target ");
    CONSOLE.print("cur mowline dist: "); 
    CONSOLE.print(distToPath); 
    CONSOLE.print(" ");
    CONSOLE.print(dst.x());
    CONSOLE.print(",");
    CONSOLE.println(dst.y());

    newTargetPoint.assign(dst);
    return true;
  }

  CONSOLE.print("findObstacleSafeMowPoint obstacle target ");    
  CONSOLE.print("cur mowline dist: "); 
  CONSOLE.print(distToPath); 
  CONSOLE.print(" ");
  CONSOLE.print(bestsec.x());
  CONSOLE.print(",");
  CONSOLE.println(bestsec.y());

  newTargetPoint.assign(bestsec);
  return true;
}

bool Map::mowingCompleted(){
  return (mowPointsIdx >= mowPoints.numPoints-1);
} 

// find start point for path finder on line from src to dst
// that is insider perimeter and outside exclusions
bool Map::checkpoint(float x, float y){
  Point src;
  src.setXY(x, y);
  if (!maps.pointIsInsidePolygon( maps.perimeterPoints, src)){
    return true;
  }
  for (int i=0; i < maps.exclusions.numPolygons; i++){
    if (maps.pointIsInsidePolygon( maps.exclusions.polygons[i], src)){
       return true;
    }
  } 
  for (int i=0; i < obstacles.numPolygons; i++){
    if (maps.pointIsInsidePolygon( maps.obstacles.polygons[i], src)){
       return true;
    }
  }  

  return false;
}

// find start point for path finder on line from src to dst
// that is insider perimeter and outside exclusions
void Map::findPathFinderSafeStartPoint(Point &src, Point &dst){
  CONSOLE.print("findPathFinderSafePoint (");  
  CONSOLE.print(src.x());
  CONSOLE.print(",");
  CONSOLE.print(src.y());
  CONSOLE.print(") (");
  CONSOLE.print(dst.x());
  CONSOLE.print(",");
  CONSOLE.print(dst.y());
  CONSOLE.println(")");  
  Point sect;
  if (!pointIsInsidePolygon( perimeterPoints, src)){
    if (linePolygonIntersectPoint( src, dst, perimeterPoints, sect)){
      src.assign(sect);
      CONSOLE.println("found safe point inside perimeter");
      return;
    }    
  }
  for (int i=0; i < exclusions.numPolygons; i++){
    if (pointIsInsidePolygon( exclusions.polygons[i], src)){
      if (linePolygonIntersectionCount(src, dst, exclusions.polygons[i]) == 1){
        // source point is not reachable      
        if (linePolygonIntersectPoint( src, dst, exclusions.polygons[i], sect)){    
          src.assign(sect);
          CONSOLE.println("found safe point outside exclusion");
          return;
        }
      }
    }
  }  
  // point is inside perimeter and outside exclusions
  CONSOLE.println("point is inside perimeter and outside exclusions");
}


// go to next point
// sim=true: only simulate (do not change data)
//
// special handling on Linux
// WAY_MOW creates a new WAY_FREE list to have intermediatepoints
// between src and new mowpoint
// may be due to an obstacle on the way from src to dst
// if WAY_FREE ends - it calls WAY_MOW again to get a new WAY_FREE
// list
//
// 1 found path success
// -1 still in progress
// 0 failed path / no path found
unsigned int Map::nextPoint(bool sim,float stateX, float stateY, bool nextmowpoint){
  CONSOLE.print("nextPoint sim=");
  CONSOLE.print(sim);
  CONSOLE.print(" wayMode=");
  CONSOLE.print(wayMode);
  CONSOLE.print(" mowpointidx=");
  CONSOLE.println(mowPointsIdx);
  if (wayMode == WAY_DOCK){
    return (nextDockPoint(sim, nextmowpoint)) ? 1 : 0;
  } 
  else if (wayMode == WAY_MOW) {
#ifndef __linux__
    return (nextMowPoint(sim, nextmowpoint)) ? 1 : 0;
#else
    Point src;
    Point dst;

    if (sim) {
      // if we run in sim mode - skip this code
      return (nextMowPoint(sim, nextmowpoint));
    }

    if (!nextPointinProgress) {
      nextPointState.setXY(stateX, stateY);
    }
    nextPointinProgress = true;

    src.setXY(stateX, stateY);

    // check if src is inside obstacle - this will prevent us from finding a path
    int ob_idx = isPointInsideObstacle(src, -1);
    // src is inside obstacle - this might be problematic for finding a path
    if ( ob_idx != -1 ) {
      CONSOLE.println("Map::nextPoint: WARN: src is inside obstacle - remove obstacle!");
      obstacles.removePolygon(ob_idx);
    }

    // only skip to next mowpoint if nextmowpoint is set true
    // this should normaly only happen by linetracker in WAY_FREE mode
    if (!nextMowPoint(false, nextmowpoint)){
      CONSOLE.println("Map::nextPoint: ERROR: no more mowing points!");
      return false;
    }

    // this loop has currently no sense - we might want to implement
    // some special logic if findPath fails - like skip to next point...
    if (!findObstacleSafeMowPoint(dst, nextPointState.x(), nextPointState.y())) {
      CONSOLE.println("Map::nextPoint: ERROR: no safe mow point found!");
      return false;
    }
    if (!findPath(src, dst)) {

      // if we're searching for next point try again?
      if (nextmowpoint) {
        CONSOLE.println("Map::nextPoint: WARN: needed to skip point!");
        return nextPoint(sim, stateX, stateY, true);
      }

      // skip current dst point by setting state to current dst for searching new dst
      // but keep state for findpath
      CONSOLE.println("Map::nextPoint: WARN: no path found - move to next step!");
      nextPointState.setXY(dst.x(), dst.y());
    } else {
      nextPointinProgress = false;
      // move to WAY_FREE list
      wayMode = WAY_FREE;

      return 1;
    }
    wayMode = WAY_FREE;

    CONSOLE.print("nextPoint sim=");
    CONSOLE.print(sim);
    CONSOLE.print(" wayMode now=");
    CONSOLE.print(wayMode);
    CONSOLE.print(" mowpointidx=");
    CONSOLE.println(mowPointsIdx);

    // still in progress
    return -1;
#endif
  } 
  else if (wayMode == WAY_FREE) {
    bool r = nextFreePoint(sim, nextmowpoint);
    // if WAY_FREE ended - it switches to WAY_MOW - reschedule function
    if (wayMode == WAY_MOW && !sim) {
      CONSOLE.println("nextFreePoint ended and is now WAY_MOW again");
      return nextPoint(sim, stateX, stateY, nextmowpoint);
    }
    return r ? 1 : 0;
  } else return 0;
}


// get next mowing point
bool Map::nextMowPoint(bool sim, bool nextpoint){  
  if (!nextpoint) {
    return true;
  }
  if (shouldMow){
    if (mowPointsIdx+1 < mowPoints.numPoints){
      // next mowing point       
      if (!sim) lastTargetPoint.assign(targetPoint);
      if (!sim) mowPointsIdx++;
      //if (!sim) targetPointIdx++;      
      return true;
    } else {
      // finished mowing;
      mowPointsIdx = 0;            
      return false;
    }         
  } else if ((shouldDock) && (dockPoints.numPoints > 0)) {      
      // go docking
      if (!sim) lastTargetPoint.assign(targetPoint);
      if (!sim) freePointsIdx = 0; 
      if (!sim) wayMode = WAY_FREE;      
      return true;    
  } else return false;  
}

// get next docking point  
bool Map::nextDockPoint(bool sim, bool nextpoint){    
  if (!nextpoint) {
    return true;
  }
  if (shouldDock){
    // should dock  
    if (dockPointsIdx+1 < dockPoints.numPoints){
      if (!sim) { 
        lastTargetPoint.assign(targetPoint);
        if (dockPointsIdx == 0) {
          CONSOLE.println("nextDockPoint: shouldRetryDock=false");
          shouldRetryDock=false;
        }
        if (shouldRetryDock) {
          CONSOLE.println("nextDockPoint: shouldRetryDock=true");
          dockPointsIdx--;
          trackReverse = true;                    
        } else {
          dockPointsIdx++; 
          trackReverse = false;                            
        }
      }              
      if (!sim) trackSlow = true;
      if (!sim) useGPSfixForPosEstimation = true;
      if (!sim) useGPSfixForDeltaEstimation = true;      
      if (!sim) useGPSfloatForPosEstimation = false;    
      if (!sim) useGPSfloatForDeltaEstimation = false;    
      if (!sim) useIMU = true;     // false      
      return true;
    } else {
      // finished docking
      return false;
    } 
  } else if (shouldMow){
    // should undock
    if (dockPointsIdx > 0){
      if (!sim) lastTargetPoint.assign(targetPoint);
      if (!sim) dockPointsIdx--;              
      if (!sim) {
        trackReverse = (dockPointsIdx >= dockPoints.numPoints-2) ; // undock reverse only in dock
      }              
      if (!sim) trackSlow = true;      
      return true;
    } else {
      // finished undocking
      if ((shouldMow) && (mowPoints.numPoints > 0 )){
        if (!sim) lastTargetPoint.assign(targetPoint);
        //if (!sim) targetPointIdx = freeStartIdx;
        if (!sim) wayMode = WAY_FREE;      
        if (!sim) trackReverse = false;              
        if (!sim) trackSlow = false;
        if (!sim) useGPSfixForPosEstimation = true;        
        if (!sim) useGPSfixForDeltaEstimation = true;
        if (!sim) useGPSfloatForPosEstimation = true;    
        if (!sim) useGPSfloatForDeltaEstimation = true;    
        if (!sim) useIMU = true;    
        return true;
      } else return false;        
    }  
  }
  return false;
}

// get next free point  
bool Map::nextFreePoint(bool sim, bool nextpoint){
  if (!nextpoint) {
    return true;
  }
  // free points
  if (freePointsIdx+1 < freePoints.numPoints){
    if (!sim) lastTargetPoint.assign(targetPoint);
    if (!sim) freePointsIdx++;                  
    return true;
  } else {
    // finished free points
    if ((shouldMow) && (mowPoints.numPoints > 0 )){
      // start mowing
      if (!sim) lastTargetPoint.assign(targetPoint);      
      if (!sim) wayMode = WAY_MOW;
      return true;  
    } else if ((shouldDock) && (dockPoints.numPoints > 0)){      
      // start docking
      if (!sim) lastTargetPoint.assign(targetPoint);
      if (!sim) dockPointsIdx = 0;      
      if (!sim) wayMode = WAY_DOCK;      
      return true;
    } else return false;
  }  
}

void Map::setLastTargetPoint(float stateX, float stateY){
  lastTargetPoint.setXY(stateX, stateY);
}


// ------------------------------------------------------ 


float Map::distance(Point &src, Point &dst) {
  return sqrt(sq(src.x()-dst.x())+sq(src.y()-dst.y()));  
}


// checks if point is inside bounding box given by points A, B
bool Map::isPointInBoundingBox(Point &pt, Point &A, Point &B){
  float minX = min(A.x(), B.x());
  float minY = min(A.y(), B.y());
  float maxX = max(A.x(), B.x());
  float maxY = max(A.y(), B.y());    
  if (pt.x() < minX-0.02) return false;
  if (pt.y() < minY-0.02) return false;
  if (pt.x() > maxX+0.02) return false;
  if (pt.y() > maxY+0.02) return false;
  return true;
}

// calculates intersection point (or touch point) of two lines 
// (A,B) 1st line
// (C,D) 2nd line  
// https://www.geeksforgeeks.org/program-for-point-of-intersection-of-two-lines/
bool Map::lineLineIntersection(Point &A, Point &B, Point &C, Point &D, Point &pt)  { 
  //console.log('lineLineIntersection', A,B,C,D);
  if ((distance(A, C) < 0.02) || (distance(A, D) < 0.02)) { 
    pt.assign(A);   
    return true;
  } 
  if ((distance(B, C) < 0.02) || (distance(B, D) < 0.02)){
    pt.assign(B);
    return true;
  }   
  // Line AB represented as a1x + b1y = c1 
  float a1 = B.y() - A.y(); 
  float b1 = A.x() - B.x(); 
  float c1 = a1*(A.x()) + b1*(A.y()); 
  // Line CD represented as a2x + b2y = c2 
  float a2 = D.y() - C.y(); 
  float b2 = C.x() - D.x(); 
  float c2 = a2*(C.x())+ b2*(C.y());   
  float determinant = a1*b2 - a2*b1;   
  if (determinant == 0)  { 
      // The lines are parallel.         
      //console.log('lines are parallel');
      return false;
  } else { 
      float x = (b2*c1 - b1*c2)/determinant; 
      float y = (a1*c2 - a2*c1)/determinant;             
      Point cp;
      cp.setXY(x, y);    
      if (!isPointInBoundingBox(cp, A, B)) return false; // not in bounding box of 1st line
      if (!isPointInBoundingBox(cp, C, D)) return false; // not in bounding box of 2nd line
      pt.assign(cp);
      return true;
  } 
} 

    

// determines if a line intersects (or touches) a polygon and returns shortest intersection point from src
bool Map::linePolygonIntersectPoint( Point &src, Point &dst, Polygon &poly, Point &sect) {      
  //Poly testpoly = poly;
  //if (allowtouch) testpoly = this.polygonOffset(poly, -0.02);      
  Point p1;
  Point p2;
  Point cp;
  float minDist = 9999;
  //CONSOLE.print("linePolygonIntersectPoint (");
  //CONSOLE.print(src.x());  
  //CONSOLE.print(",");
  //CONSOLE.print(src.y());
  //CONSOLE.print(") (");
  //CONSOLE.print(dst.x());
  //CONSOLE.print(",");
  //CONSOLE.print(dst.y());
  //CONSOLE.println(")");
  for (int i = 0; i < poly.numPoints; i++) {      
    p1.assign( poly.points[i] );
    p2.assign( poly.points[ (i+1) % poly.numPoints] );                
    //CONSOLE.print("(");
    //CONSOLE.print(p1.x());
    //CONSOLE.print(",");
    //CONSOLE.print(p1.y());
    //CONSOLE.print(") ");
    //CONSOLE.print("(");
    //CONSOLE.print(p2.x());
    //CONSOLE.print(",");
    //CONSOLE.print(p2.y());
    //CONSOLE.print(") ");
    if (lineIntersects(p1, p2, src, dst)) {        
      //CONSOLE.print(" intersect  ");
      if (lineLineIntersection(p1, p2, src, dst, cp)){        
        //CONSOLE.print(cp.x());
        //CONSOLE.print(",");
        //CONSOLE.print(cp.y());
        float dist = distance(src, cp);
        //CONSOLE.print("  dist=");
        //CONSOLE.println(dist);
        if (dist < minDist){
          minDist = dist;
          sect.assign(cp);
        }        
      }
    } // else CONSOLE.println();    
    //if (this.doIntersect(p1, p2, src, dst)) return true;
    //if (this.lineLineIntersection(p1, p2, src, dst) != null) return true;      
  }     
  return (minDist < 9999);  
}



// checks if point is inside obstacle polygon (or touching polygon line or points)
// The algorithm is ray-casting to the right. Each iteration of the loop, the test point is checked against
// one of the polygon's edges. The first line of the if-test succeeds if the point's y-coord is within the
// edge's scope. The second line checks whether the test point is to the left of the line
// If that is true the line drawn rightwards from the test point crosses that edge.
// By repeatedly inverting the value of c, the algorithm counts how many times the rightward line crosses the
// polygon. If it crosses an odd number of times, then the point is inside; if an even number, the point is outside.
bool Map::pointIsInsidePolygon( Polygon &polygon, Point &pt)
{
  int i, j, c = 0;
  int nvert = polygon.numPoints;
  if (nvert == 0) return false;
  Point pti;
  Point ptj;  
  int x = pt.px;
  int y = pt.py;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    pti.assign(polygon.points[i]);
    ptj.assign(polygon.points[j]);    
    
    #ifdef FLOAT_CALC    
    if ( ((pti.y()>pt.y()) != (ptj.y()>pt.y())) &&
     (pt.x() < (ptj.x()-pti.x()) * (pt.y()-pti.y()) / (ptj.y()-pti.y()) + pti.x()) )
       c = !c;             
    #else           
    if ( ((pti.py>y) != (ptj.py>y)) &&
     (x < (ptj.px-pti.px) * (y-pti.py) * 10 / (ptj.py-pti.py) / 10 + pti.px) )
       c = !c;
    #endif       
    
  }
  //if (c != d){
  //  CONSOLE.println("pointIsInsidePolygon bogus");
  //}
  return (c % 2 != 0);
}      


// checks if two lines intersect (or if single line points touch with line or line points)
// (p0,p1) 1st line
// (p2,p3) 2nd line
bool Map::lineIntersects (Point &p0, Point &p1, Point &p2, Point &p3) {
  /*CONSOLE.print("lineIntersects ((");
  CONSOLE.print(p0.x);  
  CONSOLE.print(",");
  CONSOLE.print(p0.y);
  CONSOLE.print("),(");
  CONSOLE.print(p1.x);
  CONSOLE.print(",");
  CONSOLE.print(p1.y);
  CONSOLE.print("))   ((");   
  CONSOLE.print(p2.x);  
  CONSOLE.print(",");
  CONSOLE.print(p2.y);
  CONSOLE.print("),(");
  CONSOLE.print(p3.x);
  CONSOLE.print(",");
  CONSOLE.print(p3.y);
  CONSOLE.print(")) ");*/  
  int p0x = p0.px;
  int p0y = p0.py;
  int p1x = p1.px;
  int p1y = p1.py;
  int p2x = p2.px;
  int p2y = p2.py;
  int p3x = p3.px;
  int p3y = p3.py;  
  int s1x = p1x - p0x;
  int s1y = p1y - p0y;
  int s2x = p3x - p2x;
  int s2y = p3y - p2y;
  
  #ifdef FLOAT_CALC
  float s = ((float) (-s1y * (p0x - p2x) + s1x * (p0y - p2y))  ) /  ((float) (-s2x * s1y + s1x * s2y)  );
  float t = ((float) (s2x * (p0y - p2y) - s2y * (p0x - p2x))   ) /  ((float)  (-s2x * s1y + s1x * s2y) );
  return ((s >= 0) && (s <= 1) && (t >= 0) && (t <= 1));
  
  #else  
  int snom = (-s1y * (p0x - p2x) + s1x * (p0y - p2y));
  int sdenom = (-s2x * s1y + s1x * s2y);
  int tnom = (s2x * (p0y - p2y) - s2y * (p0x - p2x));
  int tdenom = (-s2x * s1y + s1x * s2y);  
  
  if ( (snom < 0) && ( (sdenom > 0) || (snom < sdenom) ) ) return false;
  if ( (snom > 0) && ( (sdenom < 0) || (snom > sdenom) ) ) return false;
      
  if ( (tnom < 0) && ( (tdenom > 0) || (tnom < tdenom) ) ) return false;
  if ( (tnom > 0) && ( (tdenom < 0) || (tnom > tdenom) ) ) return false;    
  
  return true;
  #endif
  //if (res1 != res2){
  //  CONSOLE.println("lineIntersects bogus");
  //}  
}


// determines how often a line intersects (or touches) a polygon
int Map::linePolygonIntersectionCount(Point &src, Point &dst, Polygon &poly){
  Point p1;
  Point p2;
  int count = 0;
  for (int i = 0; i < poly.numPoints; i++) {      
    p1.assign( poly.points[i] );
    p2.assign( poly.points[ (i+1) % poly.numPoints] );             
    if (lineIntersects(p1, p2, src, dst)) {        
      count++;
    }  
  }       
  return count;
}
    
  
// determines if a line intersects (or touches) a polygon
bool Map::linePolygonIntersection( Point &src, Point &dst, Polygon &poly) {      
  //Poly testpoly = poly;
  //if (allowtouch) testpoly = this.polygonOffset(poly, -0.02);      
  Point p1;
  Point p2;
  for (int i = 0; i < poly.numPoints; i++) {      
    p1.assign( poly.points[i] );
    p2.assign( poly.points[ (i+1) % poly.numPoints] );             
    if (lineIntersects(p1, p2, src, dst)) {        
      return true;
    }
    //if (this.doIntersect(p1, p2, src, dst)) return true;
    //if (this.lineLineIntersection(p1, p2, src, dst) != null) return true;      
  }       
  return false;
}
      
  
// Calculates the area of a polygon.
float Map::polygonArea(Polygon &poly){
  float a = 0;
  int i;
  int l;
  Point v0;
  Point v1;
  for (i = 0, l = poly.numPoints; i < l; i++) {
    v0.assign( poly.points[i] );
    v1.assign( poly.points[i == l - 1 ? 0 : i + 1] );
    a += v0.x() * v1.y();
    a -= v1.x() * v0.y();
  }
  return a / 2;
}
  

  
  
// offset polygon points by distance
// https://stackoverflow.com/questions/54033808/how-to-offset-polygon-edges
bool Map::polygonOffset(Polygon &srcPoly, Polygon &dstPoly, float dist){    
  bool orient = (polygonArea(srcPoly) >= 0);
  //console.log('orient',orient);
  //var orient2 = ClipperLib.Clipper.Orientation(poly);
  //if (orient != orient2){
  //  console.log('polygonOffset bogus!');   
  //}        
  if (!dstPoly.alloc(srcPoly.numPoints)) return false;
  Point p1;
  Point p2;
  Point p3;  
  for (int idx1 = 0; idx1 < srcPoly.numPoints; idx1++){
    int idx2 = idx1-1;
    if (idx2 < 0) idx2 = srcPoly.numPoints-1;
    int idx3 = idx1+1;
    if (idx3 > srcPoly.numPoints-1) idx3 = 0;      
    p2.assign(srcPoly.points[idx2]); // previous           
    p1.assign(srcPoly.points[idx1]); // center
    p3.assign(srcPoly.points[idx3]); // next                 
    float a3 = atan2(p3.y() - p1.y(), p3.x() - p1.x());            
    float a2 = atan2(p2.y() - p1.y(), p2.x() - p1.x());      
    float angle = a2 + (a3-a2)/2;      
    if (a3 < a2) angle-=PI;      
    if (!orient) angle+=PI;    
    dstPoly.points[idx1].setXY( p1.x() + dist * cos(angle), p1.y() + dist * sin(angle) );     
  }  
  return true;
}

      
float Map::calcHeuristic(Point &pos0, Point &pos1) {
  return distanceManhattan(pos0, pos1);
  //return distance(pos0, pos1) ;  
}
  
  
int Map::findNextNeighbor(NodeList &nodes, PolygonList &obstacles, Node &node, int startIdx) {
  Point dbgSrcPt(4.2, 6.2);
  Point dbgDstPt(3.6, 6.8);  
  float dbgSrcDist = distance(*node.point, dbgSrcPt);
  bool verbose = false; 
  if (dbgSrcDist < 0.2){
    verbose = true;
  }
  //CONSOLE.print("start=");
  //CONSOLE.print((*node.point).x());
  //CONSOLE.print(",");
  //CONSOLE.println((*node.point).y());   
  for (int idx = startIdx+1; idx < nodes.numNodes; idx++){
    if (nodes.nodes[idx].opened) continue;
    if (nodes.nodes[idx].closed) continue;                
    if (nodes.nodes[idx].point == node.point) continue;     
    Point *pt = nodes.nodes[idx].point;            
    
    if (verbose){
      float dbgDstDist = distance(*pt, dbgDstPt);
      if (dbgDstDist < 0.2){
        CONSOLE.println("findNextNeighbor trigger debug");        
      } else verbose = false;
    }
    //if (pt.visited) continue;
    //if (this.distance(pt, node.pos) > 10) continue;
    bool safe = true;            
    Point sectPt;
    //CONSOLE.print("----check new path with all polygons---dest=");
    //CONSOLE.print((*pt).x());
    //CONSOLE.print(",");
    //CONSOLE.println((*pt).y());  
  
    // check new path with all obstacle polygons (perimeter, exclusions, obstacles)     
    for (int idx3 = 0; idx3 < obstacles.numPolygons; idx3++){             
       bool isPeri = ((perimeterPoints.numPoints > 0) && (idx3 == 0));  // if first index, it's perimeter, otherwise exclusions                           
       if (isPeri){ // we check with the perimeter?         
         //CONSOLE.println("we check with perimeter");
         bool insidePeri = pointIsInsidePolygon(obstacles.polygons[idx3], *node.point);
         if (verbose){
           CONSOLE.print("insidePeri ");
           CONSOLE.println(insidePeri);
         }
         if (!insidePeri) { // start point outside perimeter?                                                                                      
             //CONSOLE.println("start point oustide perimeter");
             if (linePolygonIntersectPoint( *node.point, *pt, obstacles.polygons[idx3], sectPt)){               
               float dist = distance(*node.point, sectPt);          
               if (verbose){
                  CONSOLE.print("dist ");
                  CONSOLE.println(dist);
               }
               if (dist > ALLOW_ROUTE_OUTSIDE_PERI_METER){ safe = false; break; } // entering perimeter with long distance is not safe                             
               if (linePolygonIntersectionCount( *node.point, *pt, obstacles.polygons[idx3]) != 1){ 
                 if (verbose) CONSOLE.println("not safe");
                 safe = false; break; 
               }
               continue;           
             } else { safe = false; break; }                                          
         }
       } else {
         //CONSOLE.println("we check with exclusion");
         bool insideObstacle = pointIsInsidePolygon(obstacles.polygons[idx3], *node.point);
         if (insideObstacle) { // start point inside obstacle?                                                                         
             if (verbose) CONSOLE.println("inside exclusion");         
             //CONSOLE.println("start point inside exclusion");          
             if (linePolygonIntersectPoint( *node.point, *pt, obstacles.polygons[idx3], sectPt)){               
               float dist = distance(*node.point, sectPt);          
               if (verbose){
                 CONSOLE.print("dist ");
                 CONSOLE.println(dist);
               }
               if (dist > ALLOW_ROUTE_OUTSIDE_PERI_METER){ 
                 if (verbose) CONSOLE.println("not safe");
                 safe = false; break; 
               } // exiting obstacle with long distance is not safe                             
               continue;           
             } else { safe = false; break; }                                          
         }
       }        
       if (linePolygonIntersection (*node.point, *pt, obstacles.polygons[idx3])){
         if (verbose) CONSOLE.println("inside intersection");
         safe = false;
         break;
       }             
    }
    //CONSOLE.print("----check done---safe=");
    //CONSOLE.println(safe);
    if (verbose){
      CONSOLE.print("safe ");
      CONSOLE.println(safe);
    }
    if (safe) {          
      //pt.visited = true;
      //var anode = {pos: pt, parent: node, f:0, g:0, h:0};          
      //ret.push(anode);
      return idx;
    }            
  }       
  return -1;
}  


// astar path finder 
// https://briangrinstead.com/blog/astar-search-algorithm-in-javascript/
bool Map::findPath(Point &src, Point &dst){
  if ((memoryCorruptions != 0) || (memoryAllocErrors != 0)){
    CONSOLE.println("ERROR findPath: memory errors");
    return false; 
  }  
  
  unsigned long nextProgressTime = 0;
  unsigned long startTime = millis();
  unsigned long time1 = 0;
  unsigned long time2 = 0;
  unsigned long time3 = 0;
  unsigned long time4 = 0;
  unsigned long time5 = 0;
  unsigned long time6 = 0;
  unsigned long time_tmp;
  CONSOLE.print("findPath (");
  CONSOLE.print(src.x());
  CONSOLE.print(",");
  CONSOLE.print(src.y());
  CONSOLE.print(") (");
  CONSOLE.print(dst.x());
  CONSOLE.print(",");
  CONSOLE.print(dst.y());
  CONSOLE.println(")");  
  
  if (ENABLE_PATH_FINDER){    
    //CONSOLE.print("path finder is enabled");      
    #ifdef FLOAT_CALC
      //CONSOLE.print(" (using FLOAT_CALC)");    
    #endif
    //CONSOLE.println();
    
    // create path-finder obstacles    
    time1 = millis();
    int idx = 0;
    if (!pathFinderObstacles.alloc(1 + exclusions.numPolygons + obstacles.numPolygons)) return false;
    
    if (freeMemory () < 5000){
      CONSOLE.println("OUT OF MEMORY");
      return false;
    }
    
    if (!polygonOffset(perimeterPoints, pathFinderObstacles.polygons[idx], 0.04)) return false;
    idx++;
    
    for (int i=0; i < exclusions.numPolygons; i++){
      if (!polygonOffset(exclusions.polygons[i], pathFinderObstacles.polygons[idx], -0.04)) return false;
      idx++;
    }      
    for (int i=0; i < obstacles.numPolygons; i++){
      if (!polygonOffset(obstacles.polygons[i], pathFinderObstacles.polygons[idx], -0.04)) return false;
      idx++;
    }  
    
    //CONSOLE.println("perimeter");
    //perimeterPoints.dump();
    //CONSOLE.println("exclusions");
    //exclusions.dump();
    //CONSOLE.println("obstacles");
    //obstacles.dump();
    //CONSOLE.println("pathFinderObstacles");
    //pathFinderObstacles.dump();
    
    // create nodes
    int allocNodeCount = exclusions.numPoints() + obstacles.numPoints() + perimeterPoints.numPoints + 2;
    CONSOLE.print ("freem=");
    CONSOLE.print(freeMemory ());    
    CONSOLE.print("  allocating nodes ");
    CONSOLE.print(allocNodeCount);
    CONSOLE.print(" (");
    CONSOLE.print(sizeof(Node) * allocNodeCount);
    CONSOLE.println(" bytes)");

    if (!pathFinderNodes.alloc(allocNodeCount)) return false;
    for (int i=0; i < pathFinderNodes.numNodes; i++){
      pathFinderNodes.nodes[i].init();
    }
    // exclusion nodes
    idx = 0;
    for (int i=0; i < exclusions.numPolygons; i++){
      for (int j=0; j < exclusions.polygons[i].numPoints; j++){    
        pathFinderNodes.nodes[idx].point = &exclusions.polygons[i].points[j];
        idx++;
      }
    }
    // obstacle nodes    
    for (int i=0; i < obstacles.numPolygons; i++){
      for (int j=0; j < obstacles.polygons[i].numPoints; j++){    
        pathFinderNodes.nodes[idx].point = &obstacles.polygons[i].points[j];
        idx++;
      }
    }
    // perimeter nodes
    for (int j=0; j < perimeterPoints.numPoints; j++){    
      pathFinderNodes.nodes[idx].point = &perimeterPoints.points[j];
      idx++;
    }     
    time1 = millis() - time1;
    // start node
    Node *start = &pathFinderNodes.nodes[idx];
    start->point = &src;
    start->opened = true;
    idx++;
    // end node
    Node *end = &pathFinderNodes.nodes[idx];
    end->point = &dst;    
    idx++;
    //CONSOLE.print("nodes=");
    //CONSOLE.print(nodes.numNodes);
    //CONSOLE.print(" idx=");
    //CONSOLE.println(idx);
    
    
    //CONSOLE.print("sz=");
    //CONSOLE.println(sizeof(visitedPoints));
    
    int timeout = 1000;    
    Node *currentNode = NULL;
    
    // CONSOLE.print ("freem=");
    // CONSOLE.println (freeMemory ());
    
    // CONSOLE.println("starting path-finder");
    while(true) {       
      if (millis() >= nextProgressTime){
        nextProgressTime = millis() + 4000;          
        // CONSOLE.print(".");
        watchdogReset();     
        resetImuTimeout();
        resetOverallMotionTimeout();
        updateGPSMotionCheckTime();
      }
      timeout--;            
      if (timeout == 0){
        CONSOLE.println("timeout");
        break;
      }
      // Grab the lowest f(x) to process next
      int lowInd = -1;
      //CONSOLE.println("finding lowest cost node...");
      for(int i=0; i<pathFinderNodes.numNodes; i++) {
        if ((pathFinderNodes.nodes[i].opened) && ((lowInd == -1) || (pathFinderNodes.nodes[i].f < pathFinderNodes.nodes[lowInd].f))) { 
          lowInd = i;
          /*CONSOLE.print("opened node i=");
          CONSOLE.print(i);
          CONSOLE.print(" x=");
          CONSOLE.print(pathFinderNodes.nodes[i].point->x());
          CONSOLE.print(" y=");
          CONSOLE.print(pathFinderNodes.nodes[i].point->y());
          CONSOLE.print(" f=");          
          CONSOLE.print(pathFinderNodes.nodes[i].f); 
          CONSOLE.print(" lowInd=");          
          CONSOLE.print(lowInd);                    
          CONSOLE.println();*/           
        }              
      }
      //CONSOLE.print("lowInd=");
      //CONSOLE.println(lowInd);
      if (lowInd == -1) break;
      currentNode = &pathFinderNodes.nodes[lowInd]; 
      // console.log('ol '+openList.length + ' cl ' + closedList.length + ' ' + currentNode.pos.X + ',' + currentNode.pos.Y);
      // End case -- result has been found, return the traced path
      if (distance(*currentNode->point, *end->point) < 0.02) break;        
      // Normal case -- move currentNode from open to closed, process each of its neighbors      
      currentNode->opened = false;
      currentNode->closed = true;
      //console.log('cn  pos'+currentNode.pos.X+','+currentNode.pos.Y);            
      //console.log('neighbors '+neighbors.length);      
      int neighborIdx = -1;
      //CONSOLE.print("currentNode ");
      //CONSOLE.print(currentNode->point->x);
      //CONSOLE.print(",");
      //CONSOLE.println(currentNode->point->y);      
      time3 = millis();
      while (true) {        
	time_tmp = millis();
        neighborIdx = findNextNeighbor(pathFinderNodes, pathFinderObstacles, *currentNode, neighborIdx);
        time2 = time2 + ( millis() - time_tmp );
	time4 = time4 + 1;
        if (neighborIdx == -1) break;
	if (time4 > 350) {
          CONSOLE.println("pathfinder: no path - fast exit");      
          return false;
	}
        Node* neighbor = &pathFinderNodes.nodes[neighborIdx];                
        
        if (millis() >= nextProgressTime){
          nextProgressTime = millis() + 4000;          
          CONSOLE.print("+");
          watchdogReset();     
        }
        //CONSOLE.print("neighbor=");
        //CONSOLE.print(neighborIdx);
        //CONSOLE.print(":");
        //CONSOLE.print(neighbor->point->x());
        //CONSOLE.print(",");
        //CONSOLE.println(neighbor->point->y());
        //this.debugPaths.push( [currentNode.pos, neighbor.pos] );
        // g score is the shortest distance from start to current node, we need to check if
        //   the path we have arrived at this neighbor is the shortest one we have seen yet
        //var gScore = currentNode.g + 1; // 1 is the distance from a node to it's neighbor
        float gScore = currentNode->g + distance(*currentNode->point, *neighbor->point);
        bool gScoreIsBest = false;
        bool found = neighbor->opened;        
        if (!found){          
          // This the the first time we have arrived at this node, it must be the best
          // Also, we need to take the h (heuristic) score since we haven't done so yet 
          gScoreIsBest = true;
          neighbor->h = calcHeuristic(*neighbor->point, *end->point);
          neighbor->opened = true;
        }
        else if(gScore < neighbor->g) {
          // We have already seen the node, but last time it had a worse g (distance from start)
          gScoreIsBest = true;
        } 
        if(gScoreIsBest) {
          // Found an optimal (so far) path to this node.   Store info on how we got here and
          //  just how good it really is...
          neighbor->parent = currentNode;
          neighbor->g = gScore;
          neighbor->f = neighbor->g + neighbor->h;
          //neighbor.debug = "F: " + neighbor.f + "<br />G: " + neighbor.g + "<br />H: " + neighbor.h;
        }
      }
      time3 = millis() - time3;
    } 

    //CONSOLE.print("finish nodes=");
    //CONSOLE.print(pathFinderNodes.numNodes);
    //CONSOLE.print(" duration=");
    //CONSOLE.println(millis()-startTime);  
    
    //if ((millis()-startTime) > 1000) {
      CONSOLE.print("time1=");
      CONSOLE.print(time1);
      CONSOLE.print(" time2=");
      CONSOLE.print(time2);
      CONSOLE.print(" time4(count)=");
      CONSOLE.print(time4);
      CONSOLE.print(" quot=");
      CONSOLE.print(int(time2/max(time4,0.00001)));
      CONSOLE.print(" time3=");
      CONSOLE.print(time3);
      CONSOLE.print(" runtime=");
      CONSOLE.println(millis()-startTime);  
    //}

    //delay(8000); // simulate a busy path finder

    resetImuTimeout();
    resetOverallMotionTimeout();
    updateGPSMotionCheckTime();

    if ((currentNode != NULL) && (distance(*currentNode->point, *end->point) < 0.02)) {
      Node *curr = currentNode;
      int nodeCount = 0;
      while(curr) {                
        nodeCount++;
        curr = curr->parent;        
      }      
      if (!freePoints.alloc(nodeCount)) return false;
      curr = currentNode;
      int idx = nodeCount-1;
      while(curr) {                                
        freePoints.points[idx].assign( *curr->point );
        // CONSOLE.print("node pt=");
        // CONSOLE.print(curr->point->x());
        // CONSOLE.print(",");
        // CONSOLE.println(curr->point->y());
        idx--;
        curr = curr->parent;                
      }            
    } else {
      // No result was found
      CONSOLE.println("pathfinder: no path");      
      return false;
      //freePoints.alloc(2);
      //freePoints.points[0].assign(src);    
      //freePoints.points[1].assign(dst);        
    }       
  } else {  // path finder not enabled (ENABLE_PATH_FINDER=false)    
    if (!freePoints.alloc(2)) return false;
    freePoints.points[0].assign(src);    
    freePoints.points[1].assign(dst);        
  }    
  freePointsIdx=0;  
  
  checkMemoryErrors();  
  resetImuTimeout();
  return true;  
}


// path finder stress test
void Map::stressTest(){  
  Point src;
  Point dst;
  float d = 30.0;
  for (int i=0 ; i < 10; i++){
    for (int j=0 ; j < 20; j++){
      addObstacle( ((float)random(d*10))/10.0-d/2, ((float)random(d*10))/10.0-d/2, (float)random(PI*2)-PI, MOT_FORWARD);
    }
    src.setXY( ((float)random(d*10))/10.0-d/2, ((float)random(d*10))/10.0-d/2 );
    dst.setXY( ((float)random(d*10))/10.0-d/2, ((float)random(d*10))/10.0-d/2 );
    findPath(src, dst);    
    clearObstacles();
  }  
  checkMemoryErrors();
}
  
  

// integer calculation correctness test
void Map::testIntegerCalcs(){  
  Point pt;
  float d = 30.0;
  for (int i=0 ; i < 5000; i++){
    pt.setXY( ((float)random(d*10))/10.0-d/2, ((float)random(d*10))/10.0-d/2 );
    pointIsInsidePolygon( perimeterPoints, pt);    
    CONSOLE.print(".");
  }
  
  Point p1;
  Point p2;
  Point p3;
  Point p4;
  for (int i=0 ; i < 5000; i++){
    p1.setXY( ((float)random(d*10))/10.0-d/2, ((float)random(d*10))/10.0-d/2 );
    p2.setXY( ((float)random(d*10))/10.0-d/2, ((float)random(d*10))/10.0-d/2 );
    p3.setXY( ((float)random(d*10))/10.0-d/2, ((float)random(d*10))/10.0-d/2 );
    p4.setXY( ((float)random(d*10))/10.0-d/2, ((float)random(d*10))/10.0-d/2 );
    lineIntersects (p1, p2, p3, p4);
    CONSOLE.print(",");
  }   
}
  
  
