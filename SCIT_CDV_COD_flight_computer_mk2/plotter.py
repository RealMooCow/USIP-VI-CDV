import os
from random import random
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.dates as mdates
import numpy as np
from datetime import datetime as dt
from matplotlib import style


def decode(hex, fmt = False): # fmt: ascii
  try: return (b''.fromhex(hex).decode("ascii") if fmt else int(hex, 16))
  except (UnicodeDecodeError, ValueError) as xcp: print(xcp); return f"[BAD HEX:{hex}]"

def convert(conversion: int) -> float:
	return ((conversion * 2.048) / 16777215) + adc_calib

SAMPLE_PAIRS = 6
BEGIN_SEEK = 35			# how many characters in the header?
VOLTAGE_HISTORY = 100 	# # of plot points to keep - reduce clutter

plt.style.use("seaborn-darkgrid")
for param in ['figure.facecolor', 'axes.facecolor', 'savefig.facecolor']:
    plt.rcParams[param] = '#293357'  # bluish dark grey

for param in ['text.color', 'axes.labelcolor', 'xtick.color', 'ytick.color']:
    plt.rcParams[param] = '0.9'  # very light grey

# fig = plt.figure()
fig, ((ax1, ax2, ax3, ax4), (ax5, ax6, ax7, ax8)) = plt.subplots(2, 4)
fig.suptitle("Payload telemetry", size=14, weight='bold',fontdict={'family': 'sans-serif',
																		'color': 'white',
																		'weight': 'ultrabold'})
fig.supylabel("Voltage, V", size=12, fontdict={'family': 'sans-serif',
																		'color': 'lightsalmon',
																		'weight': 'bold'})
fig.supxlabel("Time\n\nCLOSE WINDOW TO END RECEPTION", size=12, fontdict={'family': 'sans-serif',
																		'color': 'lightgray',
																		'weight': 'bold'})
fig.patch.set_facecolor('#212946') 
plt.tight_layout(pad=0.2)
fig.subplots_adjust(top=0.9,bottom=-2)
plt.rcParams["grid.color"] = "#2f3b64"


filename = None
where = BEGIN_SEEK

diff_linewidth = 1.05
alpha_value = 0.06

times = []
altitudes = []
adc_v1g = []
adc_v2g = []
adc_v12 = []
adc_calv = []
adc_imon = []
adc_temp = []


try:
	filename = sys.argv[1]
	adc_calib = float(sys.argv[2])
	print(f"INTAKE FILE: {filename}")
	print(f"CALIBRATION: {adc_calib}")
	print(f"PLOT HISTORY: {VOLTAGE_HISTORY}\n")
except:
	filename = "voltage_log.csv"
	adc_calib = 0.000
	print(f"MISSING PARAMS, defaults used")
	print(f"INTAKE FILE: {filename}")
	print(f"CALIBRATION: {adc_calib}\n")
	print(f"PLOT HISTORY: {VOLTAGE_HISTORY}\n")


def animate(i):
	# global where
	global where, adc_v1g, adc_v2g, adc_v12, adc_calv, adc_imon, adc_temp
	try:
		last_where = where
		begin = dt.now()
		new_times = times
		adc_v1g = adc_v1g[-VOLTAGE_HISTORY:]
		adc_v2g = adc_v2g[-VOLTAGE_HISTORY:]
		adc_v12 = adc_v12[-VOLTAGE_HISTORY:]
		adc_calv = adc_calv[-VOLTAGE_HISTORY:]
		adc_imon = adc_imon[-VOLTAGE_HISTORY:]
		adc_temp = adc_temp[-VOLTAGE_HISTORY:]
		data = open(filename,'r')
		data.seek(where)
		lines = data.read().split('\n')
		for line in lines:
			if len(line) > 1:
				try:
					time,altitude,v1g,v2g,v12,calv,imon,temp = line.split(',')[:-1]
				except:
					print("PACKET HAS MISMATCHED # OF VALUES - IGNORING. RESET MAIN ARDUINO")
				# save payload time. 
				# but for plotting, use ground time instead
				# new_times.append(dt.strptime(time, "%H:%M:%S"))
				times.append(dt.now())

				# altitudes.append(float(altitude))
				# TEST
				altitudes.append(float(altitude) + 39 + (random() - 0.5 )* 5)

				adc_v1g.append(convert(int(decode(v1g))))
				adc_v2g.append(convert(int(decode(v2g))))
				adc_v12.append(convert(int(decode(v12))))
				adc_calv.append(convert(int(decode(calv))))
				adc_imon.append(convert(int(decode(imon))))
				adc_temp.append(convert(int(decode(temp))))
		where = data.tell()
		if where == last_where:
			return print(".",end="")
		new_times = mdates.date2num(new_times)

		data.close()

		for plot in (ax1,adc_v1g,"V1G"),(ax2,adc_v2g,"V2G"),(ax3,adc_v12,"V12"),(ax4,altitudes,"ALTITUDE"),(ax5,adc_calv,"CALV"),(ax6,adc_imon,"IMON"),(ax7,adc_temp,"TEMP"),(ax8,[0] * len(adc_temp),"unused"):
			plot[0].clear()
			plot[0].set_title(plot[2])
			plot[0].xaxis_date()
			# plot[0].xaxis.set_major_formatter(mdates.DateFormatter('%I:%M:%S %p'))
			plot[0].set_autoscaley_on(True)
			
			if plot[2] == "ALTITUDE":
				plot[0].plot(times,plot[1],color="gold")
				plot[0].yaxis.set_major_locator(plt.MaxNLocator(prune='lower', nbins='auto'))
				try:
					plot[0].set_ylim([-1,max(plot[1]) * 1.05])
				except: pass
				plot[0].fill_between(times,plot[1],color="gold",alpha=0.4)
				for n in range(1, 10):
						plot[0].plot(times, plot[1],
										linewidth=2+(diff_linewidth*n),
										alpha=alpha_value,
										color="gold")
				continue
			
			plot[0].plot(times[-VOLTAGE_HISTORY-1:],plot[1][-VOLTAGE_HISTORY-1:],color='cyan' if plot[2] != "unused" else 'lightgray')
			for n in range(1, 10):
					plot[0].plot(times[-VOLTAGE_HISTORY-1:], plot[1][-VOLTAGE_HISTORY-1:],
									linewidth=2+(diff_linewidth*n),
									alpha=alpha_value,
									color='cyan' if plot[2] != "unused" else 'lightgray')
			plot[0].set_ylim([-2.1,2.1])
	
		fig.autofmt_xdate()
		# plt.title("Voltages", fontdict=title, pad=10)
		# plt.xlabel("Time Elapsed", fontdict={'size': 12, 'color': '#7d7d7d'})
		# plt.ylabel("Voltage, V", fontdict={'size': 12, 'color': '#7d7d7d'})
		print(f"\nframe updated  [{(str(dt.now() - begin).split(':')[2][:-1])} s], {len(times)} points ", end="")
	except Exception as e:
		# raise e
		print(f"\n⚠  Frame failed to render: {str(e)}")
	


print("PLOT START")
ani = animation.FuncAnimation(fig, animate, interval=500)
plt.show()