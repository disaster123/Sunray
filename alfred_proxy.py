# install via
# pip3 install --user aiohttp

import aiohttp
from aiohttp import web

import re
import gzip

new_host = "grauonline.de"
target_uri_default = "/alexwww/ardumower/ardumower-sunray-app/"

async def handle_request(request):
    async with aiohttp.ClientSession(auto_decompress=False) as session:
        target_uri = target_uri_default
        if request.method == "POST":
            target_uri = "/"
        target_url = request.url.with_scheme("http").with_host(new_host).with_port(80).with_path(target_uri + request.match_info['tail'])
        print("fetch target: " + str(target_url))
        async with session.request(request.method, target_url, headers=request.headers, data=await request.read()) as response:
            response_headers = response.headers.copy()
            # Fge CORS-Header hinzu, um Cross-Origin-Anfragen zu ermglichen
            response_headers["Access-Control-Allow-Origin"] = "*"
            response_headers.pop("Transfer-Encoding", None)
            response_headers.pop("Content-Length", None)

            content = await response.read()

            if request.url.path == '/':
                if response.headers.get('Content-Encoding') == 'gzip':
                    content = gzip.decompress(content)
                    response_headers.pop("Content-Encoding", None)

                head_start = re.search(r"<head>", content.decode(), re.IGNORECASE)

                if head_start:
                    # Text, der nach dem <head>-Tag eingefügt werden soll
                    meta_tag = "\n<meta name=apple-mobile-web-app-capable content=yes>\n<meta name=apple-mobile-web-app-status-bar-style content=default>\n<meta name=mobile-web-app-capable content=yes>\n"
            
                    # Position des <head>-Tags im HTML-Code ermitteln
                    head_start_pos = head_start.start()
                    head_end_pos = head_start.end()
            
                    # Neuen HTML-Code erstellen, mit dem eingefügten Meta-Tag
                    content = content[:head_end_pos] + meta_tag.encode() + content[head_end_pos:]
                    
                # response_headers["Content-Length"] = str(len(content))

            # Erstelle eine HTTP-Antwort an den Client
            return web.Response(status=response.status, headers=response_headers, body=content)

app = web.Application()
app.router.add_route('*', '/{tail:.*}', handle_request)
web.run_app(app)
