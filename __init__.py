from mycroft import MycroftSkill, intent_file_handler


class Robot(MycroftSkill):
    def __init__(self):
        MycroftSkill.__init__(self)

    @intent_file_handler('robot.intent')
    def handle_robot(self, message):
        self.speak_dialog('robot')


def create_skill():
    return Robot()

