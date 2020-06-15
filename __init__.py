from mycroft import MycroftSkill, intent_file_handler
import abb

class Robot(MycroftSkill):
    def __init__(self):
        MycroftSkill.__init__(self)

    @intent_file_handler('robotl.intent')
    def handle_robotl(self, message):
        self.speak_dialog('robotl')
        robot=abb.YuMi("192.168.0.100")
        robot.Connect()
        robot.InitGrippers()
        robot.MoveHome()
        position=abb.JointTarget()
        position.joints = [0,30,-60,-20,40,0,0,0,0,0,0,0]
        position.speed = abb.Speed(100)
        robot.LeftArm.MoveTo(position)
        robot.MoveHome()
    @intent_file_handler('robotr.intent')
    def handle_robotr(self, message):
        self.speak_dialog('robotr')
        robot=abb.YuMi("192.168.0.100")
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


