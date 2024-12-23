import rospy
import yaml
from std_msgs.msg import String
import sys, os
import select
import termios
import tty
import threading

class JudgeListener:
    def __init__(self, yaml_file):
        # Load the YAML file
        with open(yaml_file, 'r') as file:
            config = yaml.safe_load(file)
        
        self.correct_answer = config['correct_answer']
        self.stage = int(config['stage'])
        self.score = 0
        self.first_hit = True

        rospy.init_node('judge_listener', anonymous=True)
        rospy.Subscriber('/judge', String, self.judge_callback)

        # Publisher for /score topic
        self.score_pub = rospy.Publisher('/score', String, queue_size=10)

        self.terminal_thread = threading.Thread(target=self.modify_score)
        self.terminal_thread.daemon = True
        self.terminal_thread.start()

        self.score_publisher_thread = threading.Thread(target=self.publish_score)
        self.score_publisher_thread.daemon = True
        self.score_publisher_thread.start()

    def judge_callback(self, msg):
        if not self.first_hit:
            rospy.loginfo("Already received, skiping...")
        else:
            self.first_hit = False
            user_answer = msg.data
            
            # Compare the user answer with the correct answer
            correct_count = sum(1 for a, b in zip(user_answer, self.correct_answer) if a == b)
            
            if correct_count > 0:
                if self.stage == 1:
                    self.score += correct_count * 10
                elif self.stage == 2:
                    self.score += correct_count * 15
                else:
                    rospy.logwarn("Unsupported stage: %d" % self.stage)
                    return

                rospy.loginfo(f"Correct count:{correct_count}, score: {self.score}")
                # self.score_pub.publish(self.score)
            else:
                rospy.loginfo("Incorrect answer.")

    def modify_score(self):
        rospy.loginfo("Press 'a' to add 10, 'd' to subtract 10, 's' to submit score.")
        try:
            while not rospy.is_shutdown():
                if is_data_available():
                    key = sys.stdin.read(1)
                    if key == 'a':
                        self.score += 10
                        rospy.loginfo(f"Score increased: {self.score}")
                    elif key == 'd':
                        self.score -= 10
                        rospy.loginfo(f"Score decreased: {self.score}")
                    elif key == 's':
                        rospy.loginfo(f"Final Score submitted: {self.score}")
                        break
                    # self.score_pub.publish(self.score)
        except rospy.ROSInterruptException:
            pass
    
    def publish_score(self):
        rate = rospy.Rate(1) 
        while not rospy.is_shutdown():
            self.score_pub.publish(str(self.score))
            # rospy.loginfo(f"Score published: {self.score}")
            rate.sleep()

def is_data_available():
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    return bool(dr)

def configure_terminal():
    tty.setcbreak(sys.stdin.fileno())

def restore_terminal():
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))

if __name__ == "__main__":
    # Provide the YAML file path
    config_file = os.path.join(os.path.dirname(__file__), 'config.yaml')

    try:
        judge_listener = JudgeListener(config_file)
        configure_terminal()
        rospy.spin()
    except FileNotFoundError:
        print(f"YAML file '{config_file}' not found.")
    except KeyError as e:
        print(f"Missing key in YAML file: {e}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        restore_terminal()
