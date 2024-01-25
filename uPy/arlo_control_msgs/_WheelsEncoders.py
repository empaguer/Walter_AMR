import ustruct as struct
from ugenpy.message import Message
class WheelsEncoders(Message):
            _md5sum = "ae7edf4b550f559fc15efc7c4191da0d"
            _type = "arlo_control_msgs/WheelsEncoders"
            _has_header = False
            _full_text = '''int16 left_encoder
int16 right_encoder'''

            __slots__ = ['left_encoder','right_encoder']
            _slots_types = ['int16','int16']        
            
            def __init__(self, *args, **kwds):
            
                if args or kwds:
                    super(WheelsEncoders, self).__init__(*args, **kwds)

                    if self.int16 is None:
                        self.int16 = 0
                    if self.int16 is None:
                        self.int16 = 0
            
            def _get_types(self):
                return self._slot_types
            
            def serialize(self, buff):
                try:
                    buff.write(struct.pack('<h',self.left_encoder))
                    buff.write(struct.pack('<h',self.right_encoder))
                except Exception as e:
                    print(e)
            
            def deserialize(self, str):
                try:
                    end = 0
                    start = end
                    end += 2
                    (self.left_encoder,) = struct.unpack('<h', str[start:end])
                    start = end
                    end += 2
                    (self.right_encoder,) = struct.unpack('<h', str[start:end])
                    return self
                except Exception as e:
                    print(e)