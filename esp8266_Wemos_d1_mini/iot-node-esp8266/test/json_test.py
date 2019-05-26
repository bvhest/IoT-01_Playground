## JSON test
import ujson

# JSON works nice with Python dictionaries:
obja = {'a': 10, "bcd": "hi"}
obja['hello'] = 123
obja['live'] = True

print(obja)
print(ujson.dumps(obja))

objx = ujson.loads('["foo", {"bar":["baz", null, 1.0, 2]}]')
print(objx)
objx = ujson.loads('{"abc": 10, "def": true, "hij": [1, 2, 3]}')
print(objx)


# The following results in a JSON syntax error:
#objx = ujson.loads("{'abc': 10, 'def': true, 'hij': [1, 2, 3]}")
# Note: JSON string (and names) always use "..." - never '...'

class MyPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y

pt = MyPoint(10, 20)
print(pt)
print(ujson.dumps(pt))

# Note: Python objects are NOT (directly) convertable to JSON (v.v.)
# there are solutions - but in many cases: just use dictionaries
