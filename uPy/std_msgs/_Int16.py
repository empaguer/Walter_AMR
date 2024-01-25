import ustruct as struct
from ugenpy.message import Message


class Int16(Message):
    _md5sum = "8524586e34fbd7cb1c08c5f5f1ca0e57"
    _type = "std_msgs/Int16"
    _has_header = False
    _full_text = """int16 data"""

    __slots__ = ["data"]
    _slots_types = ["int16"]

    def __init__(self, *args, **kwds):

        if args or kwds:
            super(Int16, self).__init__(*args, **kwds)

            if self.int16 is None:
                self.int16 = 0

    def _get_types(self):
        return self._slot_types

    def serialize(self, buff):
        try:
            buff.write(struct.pack("<h", self.data))
        except Exception as e:
            print(e)

    def deserialize(self, str):
        try:
            end = 0
            start = end
            end += 2
            (self.data,) = struct.unpack("<h", str[start:end])
            return self
        except Exception as e:
            print(e)
