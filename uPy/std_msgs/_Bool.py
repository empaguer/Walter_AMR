import ustruct as struct
from ugenpy.message import Message
class Bool(Message):
            _md5sum = "8b94c1b53db61fb6aed406028ad6332a"
            _type = "std_msgs/Bool"
            _has_header = False
            _full_text = '''bool data'''

            __slots__ = ['data']
            _slots_types = ['bool']        
            
            def __init__(self, *args, **kwds):
            
                if args or kwds:
                    super(Bool, self).__init__(*args, **kwds)

                    if self.bool is None:
                        self.bool = False
            
            def _get_types(self):
                return self._slot_types
            
            def serialize(self, buff):
                try:
                    buff.write(struct.pack('<B',self.data))
                except Exception as e:
                    print(e)
            
            def deserialize(self, str):
                try:
                    end = 0
                    start = end
                    end += 1
                    (self.data,) = struct.unpack('<B', str[start:end])
                    return self
                except Exception as e:
                    print(e)