"""
Base API for physical and virtual sensors
"""
class Sensor(object):

  def __init__(self):
    self._handlers = []
  

  def subscribe(self, handler):
    if handler in self._handlers:
      raise Exception('Handler already subscribed')
    
    self._handlers.append(handler)

    return lambda: self.unsubscribe(handler)


  def unsubscribe(self, handler):
    if handler not in self._handlers:
      raise Exception('Handler not subscribed')

    self._handlers.remove(handler)


  def _notify_change(self):
    for handler in self._handlers:
      handler(self)


  @classmethod
  def mutator(cls, func):
    def wrapper(self, *args, **kwargs):
      res = func(*args, **kwargs)
      self._notify_change()
      return res

    return wrapper