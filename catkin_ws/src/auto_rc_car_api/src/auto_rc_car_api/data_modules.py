
import rospy

class BaseDataModule:
    def __init__(self):
        pass
    def get(self):
        raise NotImplementedError
    def get_time(self):
        return rospy.Time.now()


class LocalDataModule(BaseDataModule):
    def __init__(self, rospy,
                 msg_topic, msg_type):
        self.msg_topic = msg_topic
        self.msg_type = msg_type

        self.data = self.msg_type()
        self.sub = rospy.Subscriber(self.msg_topic, self.msg_type, self.data_cb)

        self.last_time = self.get_time()
    
    def data_cb(self, msg):
        self.data = msg
        self.last_time = self.get_time()

    def get(self):
        return self.data, (self.get_time() - self.last_time).to_sec()


class RemoteDataModule(BaseDataModule):
    def __init__(self, rospy, srv_topic, srv_type, extract_fn=lambda msg: msg):
        self.srv_topic = srv_topic
        self.srv_type = srv_type

        self.extract_fn = extract_fn

        print("Connecting to %s..." % self.srv_topic)
        rospy.wait_for_service(self.srv_topic)
        self.proxy = rospy.ServiceProxy(self.srv_topic, self.srv_type)
        print("Connected")

    def get(self):
        t1 = self.get_time()
        ret = self.proxy()
        t2 = self.get_time()
        return self.extract_fn(ret), (t2 - t1).to_sec()
