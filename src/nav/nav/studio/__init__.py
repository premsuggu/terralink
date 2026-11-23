"""TerraLink Studio: a browser front end for the framework (fly the UAV, watch the map grow, send the UGV).

Layout of this package (everything except `studio_node.py` is plain Python with no ROS, so it is unit-tested in tests/nav/):

  protocol.py     how data is packed for the browser (the map as compact binary, the live state as JSON)
  commands.py     what the browser may ask for, validated and limited on the server side (never trust the client)
  server.py       the tornado web + websocket server that moves those messages
  studio_node.py  the ROS 2 node that connects the server to the simulation

Design notes: docs/work-docs/nav/step16_studio_server.md
"""
