from Tkinter import *
import ttk
import time
import rospy

################################################ ros publishers
pub_human_choice = rospy.Publisher('human_choice_topic', String, queue_size=1)
pub_human_prediction = rospy.Publisher('human_prediction_topic', String, queue_size=1)
################################################ ros events
def read_msg(msg):
	message = msg.data
	label, choice, rest = message.split(",")
	items = rest.split("-")
	return label, choice, items

def robot_turn(msg):
	label, _, items = read_msg(msg)
	p1.robot(label, items)
	p1.mainframe.tkraise()

def human_turn(msg):
	label, _, items = read_msg(msg)
	p1.human(label, items)
	p1.mainframe.tkraise()

def human_predict(msg):
	label, _, items = read_msg(msg)
	p1.human_predict(label, items)
	p1.mainframe.tkraise()

def have_chosen(msg):
	label, choice, items = read_msg(msg)
	p1.chosen(label, items, choice)
	p1.mainframe.tkraise()

################################################ window events
def human_choice(choice):
	print choice

def juge(emoji):
	print emoji

def configure(event):
	p1.change_padding(root.winfo_width(),root.winfo_height())
################################################
class choice_button:
	def __init__(self,choice,frame,row,col):
		self.choice = choice
		self.button = ttk.Button(frame, text=self.choice, image=images[self.choice], compound="top",command=lambda:human_choice(self.choice)).grid(column=col, row=row, sticky=N+S+E+W)

class predict_button:
	def __init__(self,choice,frame,row,col):
		self.choice = choice
		self.button = ttk.Button(frame, text=self.choice, image=images[self.choice], compound="top",command=lambda:human_predict(self.choice)).grid(column=col, row=row, sticky=N+S+E+W)
################################################
class page:

	def __init__(self,root):
		self.mainframe = ttk.Frame(root)
		self.mainframe.grid(column=6, row=4, sticky=(N, W, E, S))
		self.mainframe.rowconfigure(4, weight=1)


	def change_padding(self,x,y):
		for child in self.mainframe.winfo_children(): child.grid_configure(padx=max(0,(x-900)/10.), pady=y/40.)

	def set_title(self, title):
		ttk.Label(self.mainframe, text=title).grid(column=2, row=0, sticky=(W,E))

	def add_emoji(self):
		ttk.Button(self.mainframe, text="good", image=good, compound="top", command=lambda:juge("good")).grid(column=1, row=4, sticky=N+S+E+W)
		ttk.Button(self.mainframe, text="lol", image=lol, compound="top", command=lambda:juge("lol")).grid(column=2, row=4, sticky=N+S+E+W)
		ttk.Button(self.mainframe, text="wtf", image=wtf, compound="top", command=lambda:juge("wtf")).grid(column=3, row=4, sticky=N+S+E+W)
		ttk.Button(self.mainframe, text="bad", image=bad, compound="top", command=lambda:juge("bad")).grid(column=4, row=4, sticky=N+S+E+W)


	def human(self, label, items):	
		self.add_emoji()
		self.set_title(label)

		ind = 0
		for item in items:
			col = ind%4 + 1
			row = ind/4 + 1
			button = choice_button(item, self.mainframe,row,col)
			ind+=1

	def human_predict(self, label, items):	
		self.add_emoji()
		self.set_title(label)

		ind = 0
		for item in items:
			col = ind%4 + 1
			row = ind/4 + 1
			button = predict_button(item, self.mainframe,row,col)
			ind+=1

	def robot(self, label,items):
		self.add_emoji()
		self.set_title(label)

		ind = 0
		for item in items:
			col = ind%4 + 1
			row = ind/4 + 1
			ttk.Button(self.mainframe, text=item, image=images[item], compound="top").grid(column=col, row=row, sticky=N+S+E+W)
			ind+=1

	def chosen(self, label, items, choice):
		self.add_emoji()
		self.set_title(label)

		ind = 0
		for item in items:
			col = ind%4 + 1
			row = ind/4 + 1
			if item==choice:
				ttk.Button(self.mainframe, text=item, image=images[item], compound="top").grid(column=col, row=row, sticky=N+S+E+W)
			else:
				ttk.Button(self.mainframe, text="", image=gray, compound="top").grid(column=col, row=row, sticky=N+S+E+W)
			ind+=1


################################################
root = Tk()
root.geometry("1000x1000")
root.bind("<Configure>", configure)
################################################
lol = PhotoImage(file="../share/lol.gif")
good = PhotoImage(file="../share/up.gif")
bad = PhotoImage(file="../share/down.gif")
wtf = PhotoImage(file="../share/wtf.gif")

pirate = PhotoImage(file="../share/pirate.gif")
gray = PhotoImage(file="../share/gray.gif")
images = {"Jack":pirate,"Nosicaa":pirate,"R1D1":pirate,"Bender":pirate}
items = ("Jack","Nosicaa","R1D1","Bender","Jack","Nosicaa")
################################################


p1 = page(root)

if __name__=="__main__":

	#rospy.init_node("interface")
	p1.mainframe.tkraise()
	root.mainloop()

	while True:
		rospy.Subscriber('human_turn_topic', String, human_turn)
		rospy.Subscriber('human_chosen_topic', String, have_chosen)
		rospy.Subscriber('human_predict_turn_topic', String, human_predict)
		rospy.Subscriber('robot_turn_topic', String, robot_turn)
		rospy.Subscriber('robot_chosen_topic', String, have_chosen)
