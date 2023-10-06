import serial
import time
import os
from serial.serialutil import SerialException
import serial.tools.list_ports
from datetime import datetime as dt
import subprocess

# requires serial & matplotlibs . install with:
# pip install pyserial matplotlib

POLL_RATE = 0.5
ADC_CALIB = 0.009
LOG_PREFIX = "TLM_LOG"

receiver = None
line = ""
log = dt.now().strftime(f"{LOG_PREFIX}_%m-%d_%H.%M.csv")

def decode(hex, fmt = False): # fmt: ascii
  try: return (b''.fromhex(hex).decode("ascii") if fmt else int(hex, 16))
  except (UnicodeDecodeError, ValueError) as xcp: print(xcp); return f"[BAD HEX:{hex}]"

def convert(conversion: int) -> float:
	return ((conversion * 2.048) / 16777215) + ADC_CALIB

ports = serial.tools.list_ports.comports()

for port, desc, hwid in sorted(ports):
	if "Serial" in desc:
		receiver = serial.Serial(port, 9600)
		break


if not receiver: 
	print("\t\t⚠\tAborted\n\nNo open COM port - check hardware connection or drivers.\n")
	os._exit(0)

try: 
	os.remove(log)		# multiple resets in same minute
except: 
	pass
with open(log, "w+") as output:
	output.write("TIME,ALT,V1G,V2G,V12,CALV,IMON,TEMP\n")

print(" ⚠\tDO NOT CLOSE WINDOW WITHOUT HOLDING CTRL-C FOR A FEW SECONDS")
subprocess.Popen(f'python plotter.py {log} {ADC_CALIB}', creationflags=subprocess.CREATE_NEW_CONSOLE)
begin = dt.now()		# start timer


try:
	while 1:
		try: line = str(receiver.readline(), "UTF-8")
		except UnicodeError as e: print("read corrupt packet - continuing")
		if line not in ["", "\n"]:
			if "\x00\x00\x00\x00\x00\x00" not in line:
				print(line,end="")
				if line[-6:] == "SYNC\r\n":
					print("SYNC - FLIGHT COMPUTER RESTARTED")
					# begin = dt.now()
					continue
				try:
					now = dt.now() - begin
					with open(log, 'a') as cache:
						cache.write(f'{str(now).split(".")[0]},{(line.split("] "))[1].rstrip()}\n')

				except Exception as e:
					# if 'RSSI' in line: pass
					if line in ["\n", ""]: pass
					print(f"PARSE ERR - {str(e)[:55]}", end=": [")
					# print(line.rstrip("), end="]\n")
					print(line)
					line = ""
			# time.sleep(POLL_RATE)

except KeyboardInterrupt:
	print("\n\t!! Stopping execution\n")
	receiver.close()
	os._exit(0)

except SerialException:
	print("\t\t⚠\tAborted\n\nReceiver was disconnected, or encountered an error.\n")
	os._exit(0)
