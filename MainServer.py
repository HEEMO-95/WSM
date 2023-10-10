from tkinter import *
import tkinter as tk 
import ttkbootstrap as ttk 
from tkintermapview import TkinterMapView
import socket
import threading
import PygameUI
from PIL import Image, ImageTk
import os
import numpy as np
import time

HOST = 'localhost'
PORT = 12125  # state socket port

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))

connected = False
gameON = False
PygameUI.lat,PygameUI.lon,PygameUI.elev,PygameUI.tilt,PygameUI.pan,PygameUI.distance,PygameUI.bearing,PygameUI.psi,PygameUI.lat1,PygameUI.lon1,PygameUI.elev1=0,0,0,0,0,0,0,0,0,0,0
lat,lon,elev,tilt,pan,distance,bearing,psi,lat1,lon1,elev1= 0,0,0,0,0,0,0,0,0,0,0
cmd_enum, cmd_param1, cmd_param2, cmd_param3 = 0 , 0 , 0 , 0

shown_markers = [50]
marker_list = []
new_marker = True

def client_handling():
	global gameON , connected ,new_marker# Program
	global lat,lon,elev,tilt,pan,distance,bearing,psi,lat1,lon1,elev1  # receive
	global cmd_enum, cmd_param1, cmd_param2, cmd_param3  # send
	server.listen()
	connected = False

	while True:

		if not connected:
			print("listening...")
			app.menu.string_value.set(value = 'connecting...')
			server_socket, address = server.accept()
			connected = True
			print('connected from: ',address) 
			setup = True

		if connected:

			# sending and recvining data from client on connection
			try:
				server_socket.send(f'{cmd_enum} , {cmd_param1} , {cmd_param2} , {cmd_param3}#'.encode('utf-8'))
				if cmd_enum >= 10:
					print(f'command sent : {cmd_enum}')
					cmd_enum,cmd_param1,cmd_param2,cmd_param3 = 0,0,0,0
			except:
				connected = False
				print('disconnected')

			try:
				msg = server_socket.recv(1024).decode('utf-8')
			except:
				connected = False
				print('disconnected')

			msg = msg.split('#')
			for data in msg:
				if data != '':
					data = data.split(',')
					lat, lon, elev, tilt, pan, distance, bearing, psi,lat1,lon1,elev1 ,target_num, target_x, target_y= ((float(data[0])),(float(data[1])),(float(data[2])),
									(float(data[3])),(float(data[4])),(float(data[5])),(float(data[6])),(float(data[7])),(float(data[8])),(float(data[9])),(float(data[10])),
									(int(data[11])),(float(data[12])),(float(data[13])))
			if msg == '':
				print('Disconnected')
				connected = False

			# setup (runs once on connection)
			if setup == True:
				current_path = os.path.join(os.path.dirname(os.path.abspath(__file__)))
				plane_img = Image.open(os.path.join(current_path, "plane.png")).resize((40, 40))
				plane_image = ImageTk.PhotoImage(plane_img)
				eye_img = Image.open(os.path.join(current_path, "eye.png")).resize((50, 50))
				eye_image = ImageTk.PhotoImage(eye_img.convert(colors=0))

				app.menu.string_value.set(value = f'Connected from {address[0]}')
				aircraft_marker = app.map_widget.set_marker(lat1, lon1,icon=plane_image)
				sight_marker = app.map_widget.set_marker(lat, lon,icon=eye_image)
				app.map_widget.set_zoom(15)
				app.map_widget.set_position(lat1, lon1)
				setup = False
				game_setup = True

			# TGP control when gamepy is lunched.

			if gameON:
				if game_setup:
					time.sleep(0.2)
					game_setup = False
				if PygameUI.msg1 == None:
					gameON = False
					game_setup = False
				else:
					cmd_enum = PygameUI.msg1
					cmd_param1 = PygameUI.msg2
					cmd_param2 = PygameUI.msg3


			# loop (looping when connected)
			PygameUI.lat,PygameUI.lon,PygameUI.elev,PygameUI.tilt,PygameUI.pan,PygameUI.distance,PygameUI.bearing,PygameUI.psi,PygameUI.lat1,PygameUI.lon1,PygameUI.elev1=lat,lon,elev,tilt,pan,distance,bearing,psi,lat1,lon1,elev1 
			
			aircraft_marker.change_icon(ImageTk.PhotoImage(plane_img.rotate(90-np.degrees(psi))))
			aircraft_marker.set_position(lat1,lon1)
			sight_marker.set_position(lat,lon)

			if target_num != 0 and target_num not in shown_markers:
				marker_list.append([target_num,target_x,target_y])
				new_marker = True


def mark_point():
	print("marking")
	global marker_list,shown_markers,new_marker
	entered_text = app.menu.entry_var.get()
	a = entered_text.split(" ")
	marker_list.append([shown_markers[-1] + 1, float(a[0]), float(a[1])])
	app.menu.entry_var.set('')
	new_marker = True

def VideoFeed():
	global gameON
	pygameGUI = threading.Thread(target=PygameUI.game, daemon=True)
	gameON = True
	print('Game command on')
	pygameGUI.start()

def slave_cmd():
	global cmd_enum,cmd_param1,cmd_param2
	cmd = app.menu.get_menu_option()
	cmd = cmd.split(',')
	print(cmd)
	cmd_enum,cmd_param1,cmd_param2 = 10,cmd[1],cmd[2]

def loiter_cmd():
	global cmd_enum,cmd_param1,cmd_param2
	cmd = app.menu.get_menu_option()
	cmd = cmd.split(',')
	print(cmd)
	cmd_enum,cmd_param1,cmd_param2 = 20,cmd[1],cmd[2]

def cont_cmd():
	global cmd_enum
	cmd_enum = 30


def marker_gen():
	global marker_list,shown_markers,new_marker
	while True:
		time.sleep(0.1)
		if new_marker:
			new_marker = False
			app.menu.refresh_marker_select(marker_list)
			for i in range(len(marker_list)):
				if marker_list[i][0] not in shown_markers:
					app.map_widget.set_marker(marker_list[i][1],marker_list[i][2],f'{marker_list[i][0]}')
					shown_markers.append(marker_list[i][0])


def button_func():
	print('a button was pressed')

def connect():
	global connected
	if not connected:
		threading.Thread(target=client_handling,daemon=True).start()


class App(ttk.Window):

	def __init__(self, title, size):

		# main setup
		super().__init__(themename='darkly')
		self.title(title)
		self.geometry(f'{size[0]}x{size[1]}')
		self.minsize(size[0],size[1])
		self.protocol("WM_DELETE_WINDOW", self.on_closing)
		# widgets 
		self.menu = Menu(self)

		self.title(title)
		self.map_widget = TkinterMapView(width=size[0]-(10+1500*0.175), height=size[1], corner_radius=0)
		self.map_widget.place(x=10+1500*0.175, y=0)
		self.map_widget.set_position(23.8859, 45.0792)
		self.map_widget.set_zoom(1)
		self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)  # google satellite


	def on_closing(self, event=0):
		self.destroy()
		exit()

	def start(self):
		self.mainloop()


class Menu(ttk.Frame):
	def __init__(self, parent):
		super().__init__(parent)
		self.place(x=0, y=0, relwidth=0.175, relheight=1)
		
		self.columnconfigure((0,1,2),weight= 1 , uniform= 'a')
		self.rowconfigure((0,1,2,3,4,5,6,7,8,9),weight= 1 , uniform= 'a')
		
		current_path = os.path.join(os.path.dirname(os.path.abspath(__file__)))
		self.p1 = PhotoImage(file=(os.path.join(current_path, "WSMICON.png")))
		self.pic = ttk.Label(parent, image=self.p1)
		self.pic.grid(row=0, column=0,sticky="nsew",columnspan=3,rowspan=3)
		
		self.selected_value = ttk.StringVar(self)
		self.selected_value.set("Select")

		self.drop_down = ttk.OptionMenu(self, self.selected_value)
		self.drop_down.grid(row=8, column=0,padx=10,pady=10,columnspan=4,sticky="nsew")

		# entry bar
		self.entry_var = tk.StringVar()
		self.entry = ttk.Entry(self, textvariable=self.entry_var, width=20)
		self.entry.grid(row=2, column=0,padx=10,pady=10,sticky="nsew",columnspan=2)

		# Create a ttk Button widget
		self.button = ttk.Button(self, text="Mark", command = mark_point)
		self.button.grid(row=2, column=3, padx=10,pady=10,sticky="nsew")

		self.string_value = tk.StringVar( value= 'Connect')
		self.button = ttk.Button(self, textvariable= self.string_value, command = connect)
		self.button.grid(row=3, column=0,sticky="nsew", columnspan=4,pady=10, padx=10)

		self.button = ttk.Button(self, text = 'Loiter', command = loiter_cmd)
		self.button.grid(row=4, column=0,sticky="nsew", columnspan=2,pady=10, padx=10)
		self.button = ttk.Button(self, text = 'carry-on', command = cont_cmd)
		self.button.grid(row=4, column=3,sticky="nsew", pady=10, padx=10)


		self.button = ttk.Button(self, text = 'Engage (CCRP)', command = button_func)
		self.button.grid(row=5, column=0,sticky="nsew", columnspan=4,pady=10, padx=10)
		self.button = ttk.Button(self, text = 'Slave TGP', command = slave_cmd)
		self.button.grid(row=6, column=0, sticky="nsew", columnspan=4,pady=10, padx=10)
		self.button = ttk.Button(self, text = 'Show TGP Video', command = VideoFeed)
		self.button.grid(row=7, column=0, sticky="nsew", columnspan=4,pady=10, padx=10)
	
	def get_menu_option(self):
		return self.selected_value.get().strip('()')
	
	def refresh_marker_select(self,new_choices):
		# Reset var and delete all old options
		self.selected_value.set("Select point marker")
		self.drop_down['menu'].delete(0, 'end')

		for choice in new_choices:
			self.drop_down['menu'].add_command(label=choice, command=tk._setit(self.selected_value, choice))


if __name__ == "__main__":
	app = App(size=(1500,700),title="WSM Big Picture")
	threading.Thread(target=marker_gen,daemon=True).start()
	app.start()
