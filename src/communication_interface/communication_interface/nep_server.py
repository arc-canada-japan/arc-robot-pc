import nep
import netifaces as ni

### TODO: Implement the NEP server

def main(args=None):
    """
        Main function to start the NEP server.
    """
    # Get the IP address of the current device
    ip = ni.ifaddresses('wlp2s0')[ni.AF_INET][0]['addr']

    # Create a NEP server
    server = nep.master(ip)

    # Start the server
    server.run()

