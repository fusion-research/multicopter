from __future__ import division

from twisted.internet import defer, reactor

import deferral
import js

@defer.inlineCallbacks
def main():
    j = js.JS(reactor)
    
    while True:
        print [v.value for k, v in sorted(j.state.value[0].iteritems())], [v for k, v in sorted(j.state.value[1].iteritems())]
        yield deferral.sleep(.01)
deferral.launch_main(main)
