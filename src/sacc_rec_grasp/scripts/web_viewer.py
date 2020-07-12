#!/usr/bin/env python
from SimpleHTTPServer import SimpleHTTPRequestHandler
from BaseHTTPServer import HTTPServer
import rospy
import os
import threading


def main():
    port = 8741
    app = "html"
    web_dir = os.path.join(os.path.dirname(__file__), app)
    os.chdir(web_dir)

    handler = SimpleHTTPRequestHandler
    rospy.loginfo('Serving {} on port {}'.format(app, port))
    server = HTTPServer(('', port), handler)
    thread = threading.Thread(target=server.serve_forever)
    thread.daemon = True
    thread.start()
    return server


if __name__ == '__main__':
    rospy.init_node("sacc_grasp_webviewer")
    server = main()


    def shutdown_hook():
        server.shutdown()


    rospy.on_shutdown(shutdown_hook)
    rospy.spin()
