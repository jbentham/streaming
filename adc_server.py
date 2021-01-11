# Cherrypy Web server for WebGL data display
# Copyright (c) Jeremy P Bentham 2021. See http://iosoft.blog for details
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# v0.01 JPB 11/1/21  First release

import os, os.path, random, string, math, cherrypy

portnum   = 8080
fifo_name = "/tmp/adc.fifo"
ymax = 2.0
npoints = 10000
nchans = 2
nresults = 0
directory = os.getcwd()

# Oscilloscope-type ADC data display
class Grapher(object):

    # Index: show oscilloscope display
    @cherrypy.expose
    def index(self):
        return cherrypy.lib.static.serve_file(directory + "/webgl_graph.html")

    # Simulated data source
    @cherrypy.expose
    def sim(self):
        global nresults
        cherrypy.response.headers['Content-Type'] = 'text/plain'
        data = npoints * [0]
        for c in range(0, npoints, nchans):
            data[c] = (math.sin((nresults*2 + c) / 20.0) + 1.2) * ymax / 4.0
            if nchans > 1:
                data[c+1] = (math.cos((nresults*2 + c) / 200.0) + 0.8) * data[c]
                data[c+1] += random.random() / 4.0
        nresults += 1
        rsp = ",".join([("%1.3f" % d) for d in data])
        return rsp

    # FIFO data source
    @cherrypy.expose
    def fifo(self):
        cherrypy.response.headers['Content-Type'] = 'text/plain'
        try:
            f = open(fifo_name, "r")
            rsp = f.readline()
            f.close()
        except:
            rsp = "No data"
        return rsp

if __name__ == '__main__':
    cherrypy.config.update({"server.socket_port": portnum, "server.socket_host": "0.0.0.0"})
    conf = {
        '/': {
            'tools.staticdir.root': os.path.abspath(directory)
        }
    }
    cherrypy.quickstart(Grapher(), '/', conf)
