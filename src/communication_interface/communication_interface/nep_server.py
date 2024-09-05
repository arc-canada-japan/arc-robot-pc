import nep
import netifaces as ni
import sys
import ipaddress


### TODO: Implement the NEP server
# echo $(ip route get 1.1.1.1 | grep -Po '(?<=dev\s)\w+' | cut -f1 -d ' ')

def main():
    """
        Main function to start the NEP server.
        It takes one argument: the IP address of the server or the interface on which IP the server should run.
    """
    ip = sys.argv[1]

    try:
        ip = ipaddress.ip_address(sys.argv[1]) # Check if the IP address is valid
    except ValueError:
        try: # Try to get the IP address from the network interface
            ip = ipaddress.ip_address(ni.ifaddresses(sys.argv[1])[ni.AF_INET][0]['addr']) 
        except Exception as e:
            print(f"An error occurred: {e}")
            sys.exit(1)

    try:
        # Create a NEP server
        server = nep.master(str(ip))
        # Start the server
        server.run()
    except KeyboardInterrupt:
        print("Server stopped.")
        server.stop()
        sys.exit(0)
    except Exception as e:
        print(f"An error occurred: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()