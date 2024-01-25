import ustruct as struct
from ugenpy.message import Message
class WheelsPower(Message):
            _md5sum = "d0b7242d8f72aed6c30a68230134b361"
            _type = "arlo_control_msgs/WheelsPower"
            _has_header = False
            _full_text = '''float32 left_power
float32 right_power'''

            __slots__ = ['left_power','right_power']
            _slots_types = ['float32','float32']        
            
            def __init__(self, *args, **kwds):
            
                if args or kwds:
                    super(WheelsPower, self).__init__(*args, **kwds)

                    if self.float32 is None:
                        self.float32 = 0.
                    if self.float32 is None:
                        self.float32 = 0.
            
            def _get_types(self):
                return self._slot_types
            
            def serialize(self, buff):
                try:
                    buff.write(struct.pack('<f',self.left_power))
                    buff.write(struct.pack('<f',self.right_power))
                except Exception as e:
                    print(e)
            
            def deserialize(self, str):
                try:
                    end = 0
                    start = end
                    end += 4
                    (self.left_power,) = struct.unpack('<f', str[start:end])
                    start = end
                    end += 4
                    (self.right_power,) = struct.unpack('<f', str[start:end])
                    return self
                except Exception as e:
                    print(e)