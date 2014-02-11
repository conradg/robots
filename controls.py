import Tkinter
import week1


class Application(Tkinter.Frame):
    def __init__(self, master):
        Tkinter.Frame.__init__(self, master)
        self.master.config()

        self.master.bind("<Left>", self.turnLeft)
        self.master.bind("<Right>", self.turnRight)
        self.master.bind("<Up>", self.goForwards)
        self.master.bind("<Down>", self.goBackwards)

        self.main_frame = Tkinter.Frame()
        self.main_frame.pack(fill="both", expand=True)
        self.pack()

    @staticmethod
    def turnLeft(event):
        print "turning left"
        week1.turn_cw(-1)

    @staticmethod
    def turnRight(event):
        print "turning right"
        week1.turn_cw(1)

    @staticmethod
    def goForwards(event):
        print "going forwards"
        week1.go(1)

    @staticmethod
    def goBackwards(event):
        print "going backwards"
        week1.go(-1)

root = Tkinter.Tk()
app = Application(root)
app.mainloop()
