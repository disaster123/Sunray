# install via
# pip3 install --user aiohttp

import aiohttp
from aiohttp import web

new_host = "grauonline.de"
target_uri = "/alexwww/ardumower/ardumower-sunray-app/"

async def handle_request(request):
    async with aiohttp.ClientSession(auto_decompress=False) as session:
        target_url = request.url.with_scheme("http").with_host(new_host).with_port(80).with_path(target_uri + request.match_info['tail'])
        # print("fetch target: " + str(target_url))
        async with session.request(request.method, target_url, headers=request.headers, data=await request.read()) as response:
            response_headers = response.headers.copy()
            # Fge CORS-Header hinzu, um Cross-Origin-Anfragen zu ermglichen
            response_headers["Access-Control-Allow-Origin"] = "*"

            # Erstelle eine HTTP-Antwort an den Client
            return web.Response(status=response.status, headers=response_headers, body=await response.read())

app = web.Application()
app.router.add_route('*', '/{tail:.*}', handle_request)
web.run_app(app)
