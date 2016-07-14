from ConfigParser import SafeConfigParser
import os


basepath = os.path.dirname(__file__)
cfg_path = os.path.abspath(os.path.join(basepath, "./config/config.conf"))


conf = SafeConfigParser()
conf.read(cfg_path)
