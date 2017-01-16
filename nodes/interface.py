#!/usr/bin/env python
#coding: utf-8

from Tkinter import *
import ttk
import time
import rospy
import thread

from std_msgs.msg import String, Empty, Header

################################################
last_message = ""

################################################ ros publishers
pub_human_choice = rospy.Publisher('human_choice_topic', String, queue_size=1)
pub_human_prediction = rospy.Publisher('human_prediction_topic', String, queue_size=1)
pub_reception = rospy.Publisher('reception_topic', String, queue_size=1)
################################################ ros events
def read_msg(msg):
	message = msg.data
	label, choice, rest, name = message.split(",")
	items = rest.split("-")
	return label, choice, items, name

def robot_turn(msg):
	global last_message
	if msg.data!=last_message:
		last_message = msg.data
		received_msg = String()
		received_msg.data = "receives: " + msg.data
		pub_reception.publish(received_msg)
		label, _, items, name = read_msg(msg)
		p1.robot(label, items, name)
		p1.mainframe.tkraise()

def human_turn(msg):
	global last_message
	if msg.data!=last_message:
		last_message = msg.data
		received_msg = String()
		received_msg.data = "receives: " + msg.data
		pub_reception.publish(received_msg)
		label, _, items, name = read_msg(msg)
		p1.human(label, items, name)
		p1.mainframe.tkraise()

def human_predict(msg):
	global last_message
	if msg.data!=last_message:
		last_message = msg.data
		received_msg = String()
		received_msg.data = "receives: " + msg.data
		pub_reception.publish(received_msg)
		label, _, items, name = read_msg(msg)
		p1.human_predict(label, items, name)
		p1.mainframe.tkraise()

def have_chosen(msg):
	global last_message
	if msg.data!=last_message:
		last_message = msg.data
		received_msg = String()
		received_msg.data = "receives: " + msg.data
		pub_reception.publish(received_msg)
		label, choice, items, name = read_msg(msg)
		p1.chosen(label, items, choice,name)
		p1.mainframe.tkraise()

def new_phrase(msg):
	global last_message
	if msg.data!=last_message:
		last_message = msg.data
		received_msg = String()
		received_msg.data = "receives: " + msg.data
		pub_reception.publish(received_msg)
		p1.telling(msg.data)
		p1.mainframe.tkraise()

def new_element(msg):
	global last_message
	if msg.data!=last_message:
		last_message = msg.data
		received_msg = String()
		received_msg.data = "receives: " + msg.data
		pub_reception.publish(received_msg)
		p1.update_trace(msg.data)
		p1.mainframe.tkraise()

################################################ for the trace

def unique(array):
	out = []
	for mec in array:
		if not mec in out:
			out.append(mec)
	return out

################################################ window events
def human_choice(choice):
	rospy.loginfo("human choice "+choice)
	msg = String()
	msg.data = choice
	pub_human_choice.publish(msg)

def human_prediction(choice):
	rospy.loginfo("human predict "+choice)
	msg = String()
	msg.data = choice
	pub_human_prediction.publish(msg)

def juge(emoji):
	rospy.loginfo("human juge "+emoji)

def configure(event):
	p1.change_padding(root.winfo_width(),root.winfo_height())
################################################
class choice_button:
	def __init__(self,choice,frame,row,col):
		self.choice = choice
		self.button = ttk.Button(frame, text=self.choice, image=images[self.choice], compound="top",command=lambda:human_choice(self.choice)).grid(column=col, row=row, sticky=N+S+E+W)

class choice_button_name:
	def __init__(self,choice,frame,row,col):
		self.choice = choice
		self.button = ttk.Button(frame, text=self.choice,command=lambda:human_choice(self.choice)).grid(column=col, row=row, sticky=N+S+E+W)

class predict_button:
	def __init__(self,choice,frame,row,col):
		self.choice = choice
		self.button = ttk.Button(frame, text=self.choice, image=images[self.choice], compound="top",command=lambda:human_prediction(self.choice)).grid(column=col, row=row, sticky=N+S+E+W)

class predict_button_name:
	def __init__(self,choice,frame,row,col):
		self.choice = choice
		self.button = ttk.Button(frame, text=self.choice,command=lambda:human_prediction(self.choice)).grid(column=col, row=row, sticky=N+S+E+W)

################################################
class page:

	def __init__(self,root):
		self.mainframe = ttk.Frame(root)
		self.mainframe.grid(column=5, row=4, sticky=(N, W, E, S))
		self.trace = []
		#self.mainframe.rowconfigure(4, weight=1)

	def change_padding(self,x,y):
		for child in self.mainframe.winfo_children(): child.grid_configure(padx=max(0,(x-900)/30.), pady=y/40.)

	def set_title(self, title):
		ttk.Label(self.mainframe, text=title).grid(column=2, row=0, sticky=(W,E))

	def add_emoji(self):
		ttk.Button(self.mainframe, text="good", image=good, compound="top", command=lambda:juge("good")).grid(column=1, row=4, sticky=N+S+E+W)
		ttk.Button(self.mainframe, text="lol", image=lol, compound="top", command=lambda:juge("lol")).grid(column=2, row=4, sticky=N+S+E+W)
		ttk.Button(self.mainframe, text="wtf", image=wtf, compound="top", command=lambda:juge("wtf")).grid(column=3, row=4, sticky=N+S+E+W)
		ttk.Button(self.mainframe, text="bad", image=bad, compound="top", command=lambda:juge("bad")).grid(column=4, row=4, sticky=N+S+E+W)

	def update_trace(self,element):
		self.trace.append(element)
		self.trace = unique(self.trace)

	def add_trace(self):
		ttk.Label(self.mainframe, text="\n".join(self.trace),wraplength=1000).grid(column=5, row=0, rowspan = 5, sticky=N+W)

	def human(self, label, items, name):
		for child in self.mainframe.winfo_children():
			child.destroy()
		self.add_emoji()
		self.add_trace()
		self.set_title(label)

		ind = 0
		for item in items:
			col = ind%4 + 1
			row = ind/4 + 1
			if name=="true":
				button = choice_button_name(item, self.mainframe,row,col)
			else:
				button = choice_button(item, self.mainframe,row,col)
			ind+=1

	def human_predict(self, label, items, name):
		for child in self.mainframe.winfo_children():
			child.destroy()
		self.add_emoji()
		self.add_trace()
		self.set_title(label)

		ind = 0
		for item in items:
			col = ind%4 + 1
			row = ind/4 + 1
			if name=="true":
				button = predict_button_name(item, self.mainframe,row,col)
			else:
				button = predict_button(item, self.mainframe,row,col)
			ind+=1

	def robot(self, label,items, name):
		for child in self.mainframe.winfo_children():
			child.destroy()
		self.add_emoji()
		self.add_trace()
		self.set_title(label)

		ind = 0
		for item in items:
			col = ind%4 + 1
			row = ind/4 + 1
			if name=="true":
				ttk.Button(self.mainframe, text=item).grid(column=col, row=row, sticky=N+S+E+W)
			else:
				ttk.Button(self.mainframe, text=item, image=images[item], compound="top").grid(column=col, row=row, sticky=N+S+E+W)
			ind+=1

	def chosen(self, label, items, choice, name):
		#for child in self.mainframe.winfo_children():
		#	child.destroy()
		self.add_emoji()
		self.add_trace()
		self.set_title(label)

		ind = 0
		for item in items:
			col = ind%4 + 1
			row = ind/4 + 1
			if name=="true":
				if item==choice:
					ttk.Button(self.mainframe, text=item).grid(column=col, row=row, sticky=N+S+E+W)
				else:
					ttk.Button(self.mainframe, text="").grid(column=col, row=row, sticky=N+S+E+W)
			else:
				if item==choice:
					ttk.Button(self.mainframe, text=item, image=images[item], compound="top").grid(column=col, row=row, sticky=N+S+E+W)
				else:
					ttk.Button(self.mainframe, text="", image=gray, compound="top").grid(column=col, row=row, sticky=N+S+E+W)
			ind+=1

	def telling(self, phrase):
		for child in self.mainframe.winfo_children():
			child.destroy()
		self.add_emoji()
		self.add_trace()
		ttk.Label(self.mainframe, text=phrase, font=("Comic Sans MS", 20, "italic"), foreground="blue", wraplength=1000).grid(column=1, columnspan=4, row=0, sticky=N+S+E+W)


################################################
root = Tk()
root.geometry("1000x1000")
root.bind("<Configure>", configure)


################################################
def ros_loop(test):
	while True:

		rospy.Subscriber('human_turn_topic', String, human_turn)
		rospy.Subscriber('human_chosen_topic', String, have_chosen)
		rospy.Subscriber('human_predict_turn_topic', String, human_predict)
		rospy.Subscriber('robot_turn_topic', String, robot_turn)
		rospy.Subscriber('robot_chosen_topic', String, have_chosen)
		rospy.Subscriber('story_telling', String, new_phrase)
		rospy.Subscriber('new_element', String, new_element)

		rospy.sleep(0.1)

	rospy.spin()


################################################

p1 = page(root)

if __name__=="__main__":

	rospy.init_node("interface")

	################################################
	test = rospy.search_param("icones")
	icone_folder = rospy.get_param(test)
	################################################
	lol = PhotoImage(file=icone_folder+"/lol.gif")
	good = PhotoImage(file=icone_folder+"/up.gif")
	bad = PhotoImage(file=icone_folder+"/down.gif")
	wtf = PhotoImage(file=icone_folder+"/wtf.gif")
	gray = PhotoImage(file=icone_folder+"/gray.gif")
	################################################

	items = {"spam","man", "woman", "robot","pirate", "detective","knight","space pioneer", "lumberjack", "prince", "wizard", "princess", "fairy","robot pirate", "robot detective","robot knight","robot space pioneer", "robot lumberjack", "robot prince", "robot princess", "robot fairy", "robot wizard","tea", "rhum", "lazer juice", "wine", "coffee", "beer", "milk","light saber", "saber", "sword", "lazer gun", "gun","spoon","planet", "forest", "kingdom", "island", "village","ghost", "alien", "monkey", "fisherman", "robot","ghost robot", "alien robot", "robot monkey", "fisherman robot","waltz","tango","polka","salsa","rock","time travelor", "scientist", "warlock", "emperor", "general", "witch","robot time travelor", "robot scientist", "robot warlock", "robot emperor", "robot general", "robot witch","trip", "poke", "badmouth", "trap", "rob", "blackmail", "terrorise","manor", "spacecraft", "laboratory", "castle","scotch","whisky","rhum","wine", "milk","blood", "robot blood"}
	images = {}

	for item in items:
		images[item] = PhotoImage(file=icone_folder+"/"+item.replace(" ", "_")+".gif")
		#images[item] = PhotoImage(file="/home/alexis/Desktop/share/"+item.replace(" ", "_")+".gif")
	################################################

	p1.mainframe.tkraise()

	'''test = String()
	test.data = "grosse phrase  tres tres tres tres tres tres tres tres tres tres tres tres tres tres tres tres tres tres tres tres tres tres tres tres tres looooooooonnnnnnnnguuuuuueeeeeee,,,"
	new_phrase(test)
	new_phrase(test)'''

	thread.start_new_thread(ros_loop, ("",))

	root.mainloop()
