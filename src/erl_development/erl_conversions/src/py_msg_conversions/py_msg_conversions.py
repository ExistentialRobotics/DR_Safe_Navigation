# Deal with Python2 / Python 3 Inconsistencies.
try:
    from StringIO import StringIO  # for Python 2
except ImportError:
    from io import BytesIO as StringIO  # for Python 3


def _to_cpp(msg):
    """Return a serialized string from a ROS message

    Parameters
    ----------
    - msg: a ROS message instance.
    """
    buf = StringIO()
    msg.serialize(buf)
    return buf.getvalue()


def _from_cpp(str_msg, cls):
    """Return a ROS message from a serialized string

    Parameters
    ----------
    - str_msg: str, serialized message
    - cls: ROS message class, e.g. sensor_msgs.msg.LaserScan.
    """
    msg = cls()
    return msg.deserialize(str_msg)

