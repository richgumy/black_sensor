import csv
from datetime import datetime
import serial
import time
import traceback

# TODO: 1 - run in parallel with cfa code and sync timing somehow? Could have a timer on the ESP that sends through a timestamp every frame (i.e. every 256 measurments?)?

def ert_serial2csv(filename, file_dir, ser_handle):
  '''
  Log raw voltage serial data to csv file
  '''
  with open(f"{file_dir}{filename}.csv", 'a',newline='\n') as csvfile:
      writer = csv.writer(csvfile, delimiter=",")
      writer.writerow([str(datetime.now())])
      t_si = time.time()
      new_frame_flag = 0
      while(1):
        if ser_handle.in_waiting > 0:
          msg = ser_handle.read_until(b'\r')
          if msg == b'A\r':
            new_frame_flag = 1
          elif new_frame_flag:
            writer.writerow([time.time() - t_si,(int(msg))])
            new_frame_flag = 0
          else:
            writer.writerow(["",(int(msg))])
    
def ert_listen(ser_handle):
  '''
  Establish connection and begin reading raw ERT data
  '''
  msg = ''
  print("LISTENING FOR ERT PCB...")
  while msg != b'A\r':
    if ser_handle.in_waiting > 0:
      # msg = ser_handle.readline()
      msg = ser_handle.read_until(b'\r')
      print(msg)
  print("BEGINNING ERT DATA CAPTURE!")
      
    
if __name__ == "__main__":
  import sys
  if len(sys.argv) < 3: 
    print("Usage: serial_to_csv <filename> <file_dir> <comport>\n e.g. python serial_to_csv test_file /in/a/relative/dir/ COM3")
  file_out = sys.argv[1] + '.csv'
  file_dir_out = sys.argv[2]
  ser_comport = sys.argv[3]
  ser = serial.Serial(f"{ser_comport}",baudrate=115200)
  
  try:
    ert_listen(ser)
    ert_serial2csv(file_out, file_dir_out, ser)

  except Exception:
    print(traceback.format_exc())

  finally:
    with open(f"{file_dir_out}{file_out}.csv", 'a',newline='') as csvfile:
      writer = csv.writer(csvfile, delimiter=",")
      writer.writerow([str(datetime.now())])
  
  