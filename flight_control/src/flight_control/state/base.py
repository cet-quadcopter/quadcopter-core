from wrapt import synchronized, decorator



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


  @decorator
  @classmethod
  def mutator(cls, func, instance, args, kwargs):
    res = func(*args, **kwargs)
    instance._notify_change()
    
    return res
  


class DataAccumulator(object):

  def __init__(self):
    self._reset()


  @synchronized
  def post(self, data):
    self._data = self._data + data if self._data is not None else data
    self._count = self._count + 1


  @synchronized
  def get(self):
    if self._data is None:
      return None
    
    acc = self._data / self._count
    self._reset()
    return acc

  
  def _reset(self):
    self._data = None
    self._count = 0
  


class TopicCombiner(object):

  def __init__(self, handler, topics):
    self._topic_registry = set(topics)
    self._handler = handler

    self._reset()


  @synchronized
  def post(self, topic, message):
    assert topic in self._topic_registry

    self._messages[topic] = message

    if len(self._messages) == len(self._topic_registry):
      self._handler(**self._messages)
      self._reset()
  

  def _reset(self):
    self._messages = {}
