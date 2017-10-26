from __future__ import division

import sys
import struct
import json
import socket

import p2pool
p2pool.DEBUG = 0 # in order to avoid assertion failures from unpacking non-canonical data
from p2pool.util import pack



class JSON(pack.Type):
    def __init__(self, inner):
        self._inner = inner
    
    def read(self, file):
        s, file = self._inner.read(file)
        return json.loads(s), file
    
    def write(self, file, item):
        return self._inner.write(file, json.dumps(item))

class LengthPrefixedString(pack.Type):
    _inner_size = pack.IntType(32)
    
    def read(self, file):
        length, file = self._inner_size.read(file)
        return pack.read(file, length)
    
    def write(self, file, item):
        return self._inner_size.write(file, len(item)), item

class ComplexType(pack.Type):
    _inner = pack.StructType('<d')
    
    def read(self, file):
        real, file = self._inner.read(file)
        imag, file = self._inner.read(file)
        return complex(real, imag), file
    
    def write(self, file, item):
        c = complex(item)
        file = self._inner.write(file, c.real)
        file = self._inner.write(file, c.imag)
        return file

class ListType(pack.Type):
    _inner_size = pack.IntType(32)
    
    def __init__(self, inner):
        self._inner = inner
    
    def read(self, file):
        length, file = self._inner_size.read(file)
        res = []
        for i in xrange(length):
            x, file = self._inner.read(file)
            res.append(x)
        return res, file
    
    def write(self, file, item):
        file = self._inner_size.write(file, len(item))
        for i in item:
            file = self._inner.write(file, i)
        return file

class TupleType(pack.Type):
    def __init__(self, contents):
        self._contents = contents
    def read(self, file):
        res = []
        for c in self._contents:
            x, file = c.read(file)
            res.append(x)
        return tuple(res), file
    def write(self, file, item):
        res = []
        assert len(item) == len(self._contents)
        for c, x in zip(self._contents, item):
            file = c.write(file, x)
            res.append(x)
        return file

class DataType(object):
    def __init__(self, **kwargs):
        for k, v in kwargs.iteritems():
            setattr(self, k, v)
    
    @classmethod
    def _unpack(cls, id_, timestamp, data):
        assert cls._ID == id_
        x = cls.desc._unpack(data)
        res = cls()
        res.timestamp = timestamp
        for k, v in dict(x).iteritems():
            assert k != 'timestamp'
            setattr(res, k, v)
        return res
    
    @classmethod
    def unpack_prefixed(cls, read_func):
        id_, timestamp, size = struct.unpack('<BqI', read_func(13))
        data = read_func(size)
        ts = [x for x in cls.__subclasses__() if hasattr(x, '_ID') and x._ID == id_]
        #if not ts:
        #    ts = [UnknownPacket]
        t, = ts
        return t._unpack(id_, timestamp, data)
    
    @classmethod
    def unpack(cls, id_, timestamp, data):
        ts = [x for x in cls.__subclasses__() if hasattr(x, '_ID') and x._ID == id_]
        #if not ts:
        #    ts = [UnknownPacket]
        t, = ts
        return t._unpack(id_, timestamp, data)
    
    def __repr__(self):
        return '%s(%s)' % (self.__class__.__name__, ', '.join('%s=%s' % (k, repr(v)) for k, v in self.__dict__.iteritems()))

class UnknownPacket(DataType):
    @classmethod
    def _unpack(cls, id_, timestamp, data):
        res = cls()
        res.id_ = id_
        res.timestamp = timestamp
        res.data = data
        return res

class ConfigPacket(DataType):
    _ID = 1
    desc = pack.ComposedType([
        ('config', JSON(LengthPrefixedString())),
    ])

class GNSSSamplePacket(DataType):
    _ID = 2
    desc = pack.ComposedType([
        ('stream', pack.IntType(16)),
        ('sample_index', pack.IntType(64)),
        ('sample_data', LengthPrefixedString()),
    ])

class IMUPacket(DataType):
    _ID = 3
    desc = pack.ComposedType([
        ('index', pack.IntType(64)),
        ('proper_acceleration', TupleType([pack.StructType('<d')]*3)),
        ('angular_velocity', TupleType([pack.StructType('<d')]*3)),
    ])

class BarometerPacket(DataType):
    _ID = 4
    desc = pack.ComposedType([
        ('stream', pack.IntType(16)),
        ('index', pack.IntType(64)),
        ('pressure', pack.StructType('<d')),
    ])

class MagnetometerPacket(DataType):
    _ID = 6
    desc = pack.ComposedType([
        ('stream', pack.IntType(16)),
        ('index', pack.IntType(64)),
        ('magnetic_field', TupleType([pack.StructType('<d')]*3)),
    ])

# really part of cooked protocol, but we're combining them here

class GNSSCorrelationPacket(DataType):
    _ID = 128
    desc = pack.ComposedType([
        ('stream', pack.IntType(32)),
        ('substream', pack.IntType(32)),
        ('sv', pack.IntType(16)),
        ('center_samples', pack.IntType(64)), # XXX is signed
        ('center_fractional_samples', pack.StructType('<d')),
        ('doppler', pack.StructType('<d')),
        ('chip_speed', pack.StructType('<d')),
        ('correlations', ListType(ComplexType())),
    ])

# really part of observables protocol, but we're combining them here

class GNSSObservationPacket(DataType):
    _ID = 192
    desc = pack.ComposedType([
        ('stream', pack.IntType(32)),
        ('substream', pack.IntType(32)),
        ('sv', pack.IntType(16)),
        ('origin_time', pack.ComposedType([
            ('WN', pack.StructType('<i')),
            ('TOW_ps', pack.StructType('<q')),
        ])),
        ('origin_position', TupleType([pack.StructType('<d')]*3)),
        ('origin_velocity', TupleType([pack.StructType('<d')]*3)),
        ('phase', pack.StructType('<d')),
        ('doppler', pack.StructType('<d')),
        ('center_samples', pack.IntType(64)),
        ('center_fractional_samples', pack.StructType('<d')),
        ('C_over_N_0', pack.StructType('<d')),
    ])

def easy_reader():
    if len(sys.argv[1:]) == 1:
        f = open(sys.argv[1], 'rb')
        def read(n):
            res = f.read(n)
            if len(res) != n:
                raise StopIteration()
            return res
    else:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((sys.argv[1], int(sys.argv[2])))
        
        def read(n):
            res = ""
            while len(res) < n:
                d = s.recv(n - len(res))
                if not d:
                    raise StopIteration()
                res += d
            assert len(res) == n
            return res
    
    while True:
        msg = DataType.unpack_prefixed(read)
        yield msg
