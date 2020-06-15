from mycroft import MycroftSkill, intent_handler
from . import abb

class Robot(MycroftSkill):
    def __init__(self):
        MycroftSkill.__init__(self)
    
    @intent_handler('robotl.intent')
    def handle_robotl(self, message):
        self.speak_dialog('robotl')
        robot=abb.YuMi("158.193.224.163")
        robot.Connect()
        robot.InitGrippers()
        robot.MoveHome()
        position=abb.JointTarget()
        position.joints = [0,30,-60,-20,40,0,0,0,0,0,0,0]
        position.speed = abb.Speed(100)
        robot.LeftArm.MoveTo(position)
        robot.MoveHome()
    
    @intent_handler('robotr.intent')
    def handle_robotr(self, message):        
        self.speak_dialog('robotr')
        robot=abb.YuMi("158.193.224.163")
        robot.Connect()
        robot.InitGrippers()
        robot.MoveHome()
        position=abb.JointTarget()
        position.joints = [0,30,-60,-20,40,0,0,0,0,0,0,0]
        position.speed = abb.Speed(100)
        robot.RightArm.MoveTo(position)
        robot.MoveHome()
        
def create_skill():
    return Robot()
