# PseudoTcpSharp
## What this port is about
This is a simple port of PseudoTcp from C to C#. Don't expect fancy C# code, it is really trying to stick to the original, just in case we need to debug the code or add patches from C. So, it is mostly C# looking like C. But it works :-)

There is another branch (which will be eventually merged up to master) where you can find a domesticated refactor with code starting to look more C#-ish.

I basically took https://github.com/libnice/libnice/blob/master/agent/pseudotcp.c and ported it to C# and created a small test to make sure it works.

The test is based on https://github.com/libnice/libnice/blob/master/tests/test-pseudotcp.c and simply ported to C#. It doesn't use a real socket underneath, just simulates sending data and droping packets as the original code does.

## Why PseudoTcpSharp is interesting?
PseudoTcp is a TCP implementation on top of UDP. It is the thing that LibJingle and LibNice and WebRTC use to do P2P data transfer.

So, you first do UDP NAT hole punching, and once you traversed the firewalls, you use PSeudoTcp on top to have a stream-based connection and do comfortable data transfer.

I coded p2pcopy http://blog.plasticscm.com/2016/10/p2pcopy-c-console-app-to-transfer-files.html based on UDT (check the blogpost) and I found out that it would be good to add PseudoTcp support instead of UDT to add Linux support (UDT works on Linux too, but the C# bindings are not available, and on top of that, UDT is meant to be used with high-latency, so PseudoTcp is supposed to perform better).

## What is next?
* Clean the code to make it more C#-ish.
* Implement and test real UDP socket code using PSeudoTcp => so far the test simulates data transfer, but no network is involved yet.
