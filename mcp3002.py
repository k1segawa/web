import webiopi
from webiopi.devices.analog.mcp3x0x import MCP3002

webiopi.setDebug()

mcp = MCP3002()

while 1:
	ch0 = mcp.analogRead(0)
	webiopi.debug( "Ch0 : " + str(ch0) )
	ch1 = mcp.analogRead(1)
	webiopi.debug( "Ch1 : " + str(ch1) )

	webiopi.sleep( 1.0 )
