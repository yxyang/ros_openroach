#!/usr/bin/python

import crcmod
import serial
import threading
import Queue
import struct
import time

crc8 = crcmod.mkCrcFun(0x1D5,initCrc=0,rev=False)

header_format = 'BBcBI'
header_fields = ['start','length','type','flags','sequence']

MARKER_N_PIXEL = 6;

data_formats = {
  'C': {
    'format': 'ff',
    'fields': ['left','right']},
  'S': {
    'format': 'Iffffffiiffff',
    'fields': ['time','ax','ay','az','gx','gy','gz','el','er','vl','vr','voltage','current']},
  'G': {'format':'i', 'fields':['period']},
  'R': {'format':'', 'fields':[]},
  'T': {'format':'I', 'fields':['time']},
  'P': {'format':'ffff', 'fields':['velr','vell','pwmr','pwml']},
  'M': {'format':MARKER_N_PIXEL*'I', 'fields':['pixel_%d' % i for i in range(MARKER_N_PIXEL)]},
  'L': {'format':'fhh', 'fields':['laser_cmd','galvo_cmd_0','galvo_cmd_1']},
}

def parse_packet(packet):
  packet_type = packet[2]
  data_format = data_formats[packet_type]['format']
  data_fields = data_formats[packet_type]['fields']
  packet_format = header_format + data_format + 'B'
  packet_fields = header_fields + data_fields + ['crc']
  packet_values = struct.unpack(packet_format,packet)
  return dict(zip(packet_fields,packet_values))

def format_packet(packet):
  data_format = data_formats[packet['type']]['format']
  data_fields = data_formats[packet['type']]['fields']
  packet_format = header_format + data_format
  packet_fields = header_fields + data_fields
  packet.update({'start':0, 'length':struct.calcsize(packet_format)+1})
  packet_values = [packet[field] for field in packet_fields]
  packet = struct.pack(packet_format, *packet_values)
  packet += chr(crc8(packet))
  return packet
  
class PacketParser(threading.Thread):
  def __init__(self, device='/dev/ttyACM0',baud=230400):
    threading.Thread.__init__(self)
    self.daemon = True

    self.serial = serial.Serial(device,baudrate=baud)
    self.packet = ''
    self.queue = Queue.Queue()
    self.sequence = 0

  def __del__(self):
    self.flush()
    self.serial.close()

  def run(self):
    while(self.serial.isOpen()):
      self.packet = self.serial.read(1)
      if self.packet == '\x00':
        self.packet += self.serial.read(1)
        length = ord(self.packet[-1])
        if length >= 9:
          self.packet += self.serial.read(length-2)
          if crc8(self.packet) == 0:
            self.queue.put(self.packet)
  
  def get(self,block=False,timeout=None):
    try:
      return parse_packet(self.queue.get(block,timeout))
    except:
      return None

  def flush(self):
    rp = self.get()
    pkts = []
    while rp is not None:
      pkts.append(rp)
      rp = self.get()
    return pkts

  def put(self, packet):
    if type(packet) is dict:
      packet.update({'sequence':self.sequence})
      packet = format_packet(packet)

    if (self.serial.isOpen()):
      self.sequence += 1
      self.serial.write(packet)

def get_time_packet(set_time = None, flags = 0):
  if set_time is None:
    set_time = time.time()
  return {
    'type': 'T',
    'flags': flags,
    'time': set_time,
    'sequence': 0,
  }
  
def get_reset_packet():
  return {
    'type': 'R',
    'flags': 0,
    'sequence': 0
  }

def get_read_packet(period = 0):
  return {
    'type': 'G',
    'flags': 0,
    'period': period,
    'sequence': 0
  }

def get_command_packet(left=0.0,right=0.0):
  return {
    'type': 'C',
    'flags': 0,
    'left': left,
    'right': right,
    'sequence': 0
  }

def get_marker_packet(colors = None):
  if colors is None:
    colors = []
  if type(colors) is tuple:
    colors = list(colors)
  if type(colors) is not list:
    colors = [colors]
  colors = colors + MARKER_N_PIXEL*[0]
  colors = colors[:MARKER_N_PIXEL]
  pkt = {
    'type': 'M',
    'flags': 0,
    'sequence': 0,
  }
  pkt.update(zip(data_formats['M']['fields'],colors[:MARKER_N_PIXEL]))
  return pkt

def get_laser_packet(laser_cmd = 0.0, galvo_cmd_0 = 0, galvo_cmd_1 = 0):
  pkt = {
    'type': 'L',
    'flags': 0,
    'sequence': 0,
    'laser_cmd': laser_cmd,
    'galvo_cmd_0': galvo_cmd_0,
    'galvo_cmd_1': galvo_cmd_1,
  }
  return pkt

if __name__ == '__main__':
  pp = PacketParser()
  pp.start()
  while(True):
    time.sleep(0.10)
    packet = pp.get(True,timeout=0.01)
    if packet:
      print packet
