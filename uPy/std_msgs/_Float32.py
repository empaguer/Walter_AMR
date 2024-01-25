import ustruct as struct
from ugenpy.message import Message


class Float32(Message):
    _md5sum = "73fcbf46b49191e672908e50842a83d4"
    _type = "std_msgs/Float32"
    _has_header = False
    _full_text = """float32 data"""

    __slots__ = ["data"]
    _slots_types = ["float32"]

    def __init__(self, *args, **kwds):

        if args or kwds:
            super(Float32, self).__init__(*args, **kwds)

            if self.float32 is None:
                self.float32 = 0.0

    def _get_types(self):
        return self._slot_types

    def serialize(self, buff):
        try:
            buff.write(struct.pack("<f", self.data))
        except Exception as e:
            print(e)

    def deserialize(self, str):
        try:
            end = 0
            start = end
            end += 4
            (self.data,) = struct.unpack("<f", str[start:end])
            return self
        except Exception as e:
            print(e)
