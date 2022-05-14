import serial
import re
import argparse
import io

ansi_escape = re.compile(r'(?:\x1B[@-Z\\-_]|[\x80-\x9A\x9C-\x9F]|(?:\x1B\[|\x9B)[0-?]*[ -/]*[@-~])')

def loop(prompt, f: io.TextIOWrapper, ser: serial.Serial):
    while True:
        serialString=ser.readline()
        sw = serialString.decode('ascii')
        # remove garbage
        sw_stripped = ansi_escape.sub('', sw).strip('\n\r').replace(args.prompt,'')
        # if empty or not valid JSON
        if not sw_stripped or sw_stripped == '':
            continue
        print(sw_stripped)
        
        if(f):
            f.write(sw_stripped)

def main(args):
    ser = serial.Serial(args.tty, baudrate=115200, timeout=3.0)
    
    f = None
    if args.out_file:
        with open(args.out_file, args.out_mode) as f:
            loop(args.prompt, f, ser)
    else:
        loop(args.prompt, None, ser)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', action='store', dest='tty', required=False, default="/dev/ttyACM0")
    parser.add_argument('-p', action='store', dest='prompt', required=False, default="CSSE4011:~$")
    parser.add_argument('-o', action='store', dest='out_file', required=False)
    parser.add_argument('-om', action='store', dest='out_mode', required=False, default="a")
    args = parser.parse_args()
    main(args)