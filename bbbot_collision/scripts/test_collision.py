from bbbot_collision.srv import CollisionCheckRequest, CollisionCheckResponse, CollisionCheck
import rospy


def main():
    rospy.init_node("bbbot_collision_test")
    rospy.wait_for_service("collision_check")

    try:
        coll_check = rospy.ServiceProxy("collision_check", CollisionCheck)
        obj = CollisionCheckRequest()
        resp = coll_check(obj)
        print "Response", resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == "__main__":
    main()
