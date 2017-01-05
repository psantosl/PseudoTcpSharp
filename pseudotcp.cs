/*
 * This file is part of the Nice GLib ICE library.
 *
 * (C) 2010, 2014 Collabora Ltd.
 *  Contact: Philip Withnall

 *
 * The contents of this file are subject to the Mozilla Public License Version
 * 1.1 (the "License"); you may not use this file except in compliance with
 * the License. You may obtain a copy of the License at
 * http://www.mozilla.org/MPL/
 *
 * Software distributed under the License is distributed on an "AS IS" basis,
 * WITHOUT WARRANTY OF ANY KIND, either express or implied. See the License
 * for the specific language governing rights and limitations under the
 * License.
 *
 * The Original Code is the Nice GLib ICE library.
 *
 * The Initial Developers of the Original Code are Collabora Ltd and Nokia
 * Corporation. All Rights Reserved.
 *
 * Contributors:
 *   Youness Alaoui, Collabora Ltd.
 *   Philip Withnall, Collabora Ltd.
 *
 * Alternatively, the contents of this file may be used under the terms of the
 * the GNU Lesser General Public License Version 2.1 (the "LGPL"), in which
 * case the provisions of LGPL are applicable instead of those above. If you
 * wish to allow use of your version of this file only under the terms of the
 * LGPL and not to allow others to use your version of this file under the
 * MPL, indicate your decision by deleting the provisions above and replace
 * them with the notice and other provisions required by the LGPL. If you do
 * not delete the provisions above, a recipient may use your version of this
 * file under either the MPL or the LGPL.
 */

/* Reproducing license from libjingle for copied code */

/*
 * libjingle
 * Copyright 2004--2005, Google Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *  3. The name of the author may not be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*#include <stdlib.h>
#include <errno.h>
#include <string.h>

#include <glib.h>

#ifndef G_OS_WIN32
#  include <arpa/inet.h>
#endif

#include "pseudotcp.h"
#include "agent-priv.h"*/

using System;
using System.Collections.Generic;

using size_t = System.UInt32;
using gint = System.Int32;
using gint32 = System.Int32;
using guint16 = System.UInt16;
using guint32 = System.UInt32;
using guint64 = System.UInt64;
using gboolean = System.Boolean;
using guint8 = System.Byte;
using gsize = System.UInt32; // it should be 64 for 64 bit platforms...

class PseudoTcp
{
    struct PseudoTcpSocketClass {
        /*GObjectClass*/ object parent_class;
    }

    //typedef struct _PseudoTcpSocketPrivate PseudoTcpSocketPrivate;


    class PseudoTcpSocket {
        /*GObject*/ object parent;
        internal PseudoTcpSocketPrivate priv;
    }

    /**
     * PseudoTcpWriteResult:
     * @WR_SUCCESS: The write operation was successful
     * @WR_TOO_LARGE: The socket type requires that message be sent atomically
     * and the size of the message to be sent made this impossible.
     * @WR_FAIL: There was an error sending the message
     *
     * An enum representing the result value of the write operation requested by
     * the #PseudoTcpSocket.
     * <para> See also: %PseudoTcpCallbacks:WritePacket </para>
     *
     * Since: 0.0.11
     */
    enum PseudoTcpWriteResult{
      WR_SUCCESS,
      WR_TOO_LARGE,
      WR_FAIL
    }

    /**
     * PseudoTcpShutdown:
     * @PSEUDO_TCP_SHUTDOWN_RD: Shut down the local reader only
     * @PSEUDO_TCP_SHUTDOWN_WR: Shut down the local writer only
     * @PSEUDO_TCP_SHUTDOWN_RDWR: Shut down both reading and writing
     *
     * Options for which parts of a connection to shut down when calling
     * pseudo_tcp_socket_shutdown(). These correspond to the values passed to POSIX
     * shutdown().
     *
     * Since: 0.1.8
     */
    enum PseudoTcpShutdown{
      PSEUDO_TCP_SHUTDOWN_RD,
      PSEUDO_TCP_SHUTDOWN_WR,
      PSEUDO_TCP_SHUTDOWN_RDWR,
    }

    // G_DEFINE_TYPE (PseudoTcpSocket, pseudo_tcp_socket, G_TYPE_OBJECT);

    //////////////////////////////////////////////////////////////////////
    // Network Constants
    //////////////////////////////////////////////////////////////////////

    const int EINVAL = 22;
    const int EMSGSIZE = 90;
    const int ECONNABORTED = 103;     /* Software caused connection abort */
    const int ENOTCONN = 107;         /* Transport endpoint is not connected */
    const int EAGAIN = 11;            /* Try again */
    const int EWOULDBLOCK = EAGAIN;   /* Operation would block */
    const int EPIPE = 32;             /* Broken pipe */
    const int ECONNRESET = 104;       /* Connection reset by peer */
    const int ETIMEDOUT = 110;        /* Connection timed out */

    // Standard MTUs
    static guint16[] PACKET_MAXIMUMS = new guint16[]{
      65535,    // Theoretical maximum, Hyperchannel
      32000,    // Nothing
      17914,    // 16Mb IBM Token Ring
      8166,   // IEEE 802.4
      //4464,   // IEEE 802.5 (4Mb max)
      4352,   // FDDI
      //2048,   // Wideband Network
      2002,   // IEEE 802.5 (4Mb recommended)
      //1536,   // Expermental Ethernet Networks
      //1500,   // Ethernet, Point-to-Point (default)
      1492,   // IEEE 802.3
      1006,   // SLIP, ARPANET
      //576,    // X.25 Networks
      //544,    // DEC IP Portal
      //512,    // NETBIOS
      508,    // IEEE 802/Source-Rt Bridge, ARCNET
      296,    // Point-to-Point (low delay)
      //68,     // Official minimum
      0,      // End of list marker
    };

    // FIXME: This is a reasonable MTU, but we should get it from the lower layer
    const int DEF_MTU = 1400;
    const int MAX_PACKET = 65532;
    // Note: we removed lowest level because packet overhead was larger!
    const int MIN_PACKET = 296;

    // (+ up to 40 bytes of options?)
    const int IP_HEADER_SIZE = 20;
    const int ICMP_HEADER_SIZE = 8;
    const int UDP_HEADER_SIZE = 8;
    // TODO: Make JINGLE_HEADER_SIZE transparent to this code?
    // when relay framing is in use
    const int JINGLE_HEADER_SIZE = 64;

    //////////////////////////////////////////////////////////////////////
    // Global Constants and Functions
    //////////////////////////////////////////////////////////////////////
    //
    //    0                   1                   2                   3
    //    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
    //    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    //  0 |                      Conversation Number                      |
    //    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    //  4 |                        Sequence Number                        |
    //    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    //  8 |                     Acknowledgment Number                     |
    //    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    //    |               |   |U|A|P|R|S|F|                               |
    // 12 |    Control    |   |R|C|S|S|Y|I|            Window             |
    //    |               |   |G|K|H|T|N|N|                               |
    //    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    // 16 |                       Timestamp sending                       |
    //    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    // 20 |                      Timestamp receiving                      |
    //    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    // 24 |                             data                              |
    //    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    //
    //////////////////////////////////////////////////////////////////////

    const uint MAX_SEQ = 0xFFFFFFFF;
    const int HEADER_SIZE = 24;

    const int PACKET_OVERHEAD = HEADER_SIZE + UDP_HEADER_SIZE + IP_HEADER_SIZE + JINGLE_HEADER_SIZE;

    // MIN_RTO = 1 second (RFC6298, Sec 2.4)
    const int MIN_RTO = 1000;
    const int DEF_RTO = 1000; /* 1 seconds (RFC 6298 sect 2.1) */
    const int MAX_RTO = 60000; /* 60 seconds */
    const int DEFAULT_ACK_DELAY = 100; /* 100 milliseconds */
    const bool DEFAULT_NO_DELAY = false;

    const int DEFAULT_RCV_BUF_SIZE = 60 * 1024;
    const int DEFAULT_SND_BUF_SIZE = 90 * 1024;

    /* NOTE: This must fit in 8 bits. This is used on the wire. */
    enum TcpOption:byte {
      /* Google-provided options: */
      TCP_OPT_EOL = 0,  /* end of list */
      TCP_OPT_NOOP = 1,  /* no-op */
      TCP_OPT_MSS = 2,  /* maximum segment size */
      TCP_OPT_WND_SCALE = 3,  /* window scale factor */
      /* libnice extensions: */
      TCP_OPT_FIN_ACK = 254,  /* FIN-ACK support */
    }


    /*
    #define FLAG_SYN 0x02
    #define FLAG_ACK 0x10
    */

    /* NOTE: This must fit in 5 bits. This is used on the wire. */
    enum TcpFlags {
      FLAG_NONE = 0,
      FLAG_FIN = 1 << 0,
      FLAG_CTL = 1 << 1,
      FLAG_RST = 1 << 2
    }

    const int CTL_CONNECT = 0;
    //#define CTL_REDIRECT  1
    const int CTL_EXTRA = 255;


    const uint CTRL_BOUND = 0x80000000;

    /* Maximum segment lifetime (1 minute).
     * RFC 793, §3.3 specifies 2 minutes; but Linux uses 1 minute, so let’s go with
     * that. */
    const int TCP_MSL = 60 * 1000;

    // If there are no pending clocks, wake up every 4 seconds
    const int DEFAULT_TIMEOUT = 4000;
    // If the connection is closed, once per minute
    const int CLOSED_TIMEOUT = (60 * 1000);
    /* Timeout after reaching the TIME_WAIT state, in milliseconds.
     * See: RFC 1122, §4.2.2.13.
     *
     * XXX: Since we can control the underlying layer’s channel ID, we can guarantee
     * delayed segments won’t affect subsequent connections, so can radically
     * shorten the TIME-WAIT timeout (to the extent that it basically doesn’t
     * exist). It would normally be (2 * TCP_MSL). */
    const int TIME_WAIT_TIMEOUT = 1;

    //////////////////////////////////////////////////////////////////////
    // Helper Functions
    //////////////////////////////////////////////////////////////////////
    /*#ifndef G_OS_WIN32
    #  define min(first, second) ((first) < (second) ? (first) : (second))
    #  define max(first, second) ((first) > (second) ? (first) : (second))
    #endif*/

    static guint32
    bound(guint32 lower, guint32 middle, guint32 upper)
    {
       return Math.Min(Math.Max(lower, middle), upper);
    }

    static gboolean
    time_is_between(guint32 later, guint32 middle, guint32 earlier)
    {
      if (earlier <= later) {
        return ((earlier <= middle) && (middle <= later));
      } else {
        return !((later < middle) && (middle < earlier));
      }
    }

    static gint32
    time_diff(guint32 later, guint32 earlier)
    {
      guint32 LAST = 0xFFFFFFFF;
      guint32 HALF = 0x80000000;

#warning originally all conversions cast to "long" in C

      if (time_is_between(earlier + HALF, later, earlier)) {
        if (earlier <= later) {
          return (gint32) (later - earlier);
        } else {
          return (gint32)(later + (LAST - earlier) + 1);
        }
      } else {
        if (later <= earlier) {
          return (gint32)(- (earlier - later));
        } else {
          return (gint32)(-(earlier + (LAST - later) + 1));
        }
      }
    }

    ////////////////////////////////////////////////////////
    // PseudoTcpFifo works exactly like FifoBuffer in libjingle
    ////////////////////////////////////////////////////////


    class PseudoTcpFifo{
      internal guint8[] buffer;
      internal gsize buffer_length;
      internal gsize data_length;
      internal gsize read_position;
    }


    static void
    pseudo_tcp_fifo_init(PseudoTcpFifo b, gsize size)
    {
      b.buffer = new byte[size];
      b.buffer_length = size;
    }

    static void
    pseudo_tcp_fifo_clear (PseudoTcpFifo b)
    {
      if (b.buffer != null)
      {
        // g_slice_free1 (b.buffer_length, b.buffer);
          
      }
      b.buffer = null;
      b.buffer_length = 0;
    }

    static gsize
    pseudo_tcp_fifo_get_buffered (PseudoTcpFifo b)
    {
      return b.data_length;
    }

    static void memcpy(byte[] dst, gsize dstPos, byte[] src, gsize srcPos, gsize size)
    {
        Buffer.BlockCopy(src, (int) srcPos, dst, (int) dstPos, (int)size);
    }

    static gboolean
    pseudo_tcp_fifo_set_capacity(PseudoTcpFifo b, gsize size)
    {
      if (b.data_length > size)
        return false;

      if (size != b.data_length) {
        guint8[] buffer = new guint8[size];
        gsize copy = b.data_length;
        gsize tail_copy = Math.Min(copy, b.buffer_length - b.read_position);

        memcpy (buffer, 0, b.buffer, b.read_position, tail_copy);
        memcpy (buffer, tail_copy, b.buffer, 0, copy - tail_copy);
        //g_slice_free1 (b.buffer_length, b.buffer);

        b.buffer = buffer;
        b.buffer_length = size;
        b.read_position = 0;
      }

      return true;
    }

    static void g_assert(bool cond)
    {
        if (cond)
            return;

        throw new Exception("g_assert");
    }

    static void
    pseudo_tcp_fifo_consume_read_data(PseudoTcpFifo b, gsize size)
    {
      g_assert (size <= b.data_length);

      b.read_position = (b.read_position + size) % b.buffer_length;
      b.data_length -= size;
    }

    static void
    pseudo_tcp_fifo_consume_write_buffer(PseudoTcpFifo b, gsize size)
    {
      g_assert (size <= b.buffer_length - b.data_length);

      b.data_length += size;
    }

    static gsize
    pseudo_tcp_fifo_get_write_remaining (PseudoTcpFifo b)
    {
      return b.buffer_length - b.data_length;
    }

    static gsize
    pseudo_tcp_fifo_read_offset (PseudoTcpFifo b, byte[] buffer, gsize bufferPos, gsize bytes,
        gsize offset)
    {
      gsize available = b.data_length - offset;
      gsize read_position = (b.read_position + offset) % b.buffer_length;
      gsize copy = Math.Min (bytes, available);
      gsize tail_copy = Math.Min(copy, b.buffer_length - read_position);

      /* EOS */
      if (offset >= b.data_length)
        return 0;

      memcpy(buffer, bufferPos, b.buffer, read_position, tail_copy);
      memcpy(buffer, tail_copy + bufferPos, b.buffer, 0, copy - tail_copy);

      return copy;
    }

    static gsize
    pseudo_tcp_fifo_write_offset (PseudoTcpFifo b, byte[] buffer,
        gsize bytes, gsize offset)
    {
      gsize available = b.buffer_length - b.data_length - offset;
      gsize write_position = (b.read_position + b.data_length + offset)
          % b.buffer_length;
      gsize copy = Math.Min (bytes, available);
      gsize tail_copy = Math.Min(copy, b.buffer_length - write_position);

      if (b.data_length + offset >= b.buffer_length) {
        return 0;
      }

      memcpy(b.buffer, write_position, buffer, 0, tail_copy);
      memcpy(b.buffer, 0, buffer, tail_copy, copy - tail_copy);

      return copy;
    }

    static gsize
    pseudo_tcp_fifo_read (PseudoTcpFifo b, byte[] buffer, gsize bytes)
    {
      gsize copy;

      copy = pseudo_tcp_fifo_read_offset (b, buffer, 0, bytes, 0);

      b.read_position = (b.read_position + copy) % b.buffer_length;
      b.data_length -= copy;

      return copy;
    }

    static gsize
    pseudo_tcp_fifo_write (PseudoTcpFifo b, byte[] buffer, gsize bytes)
    {
      gsize copy;

      copy = pseudo_tcp_fifo_write_offset (b, buffer, bytes, 0);
      b.data_length += copy;

      return copy;
    }


    //////////////////////////////////////////////////////////////////////
    // PseudoTcp
    //////////////////////////////////////////////////////////////////////

    /* Only used if FIN-ACK support is disabled. */
    enum Shutdown{
      SD_NONE,
      SD_GRACEFUL,
      SD_FORCEFUL
    }

    enum SendFlags{
      sfNone,
      sfDelayedAck,
      sfImmediateAck,
      sfFin,
      sfRst,
      sfDuplicateAck,
    }

    class Segment{
      internal guint32 conv, seq, ack;
      internal TcpFlags flags;
      internal guint16 wnd;
      internal byte[] data;
      internal guint32 len;
      internal guint32 tsval, tsecr;
    }

    class SSegment{
      internal guint32 seq, len;
      internal guint8 xmit;
      internal TcpFlags flags;
    }

    class RSegment{
      internal guint32 seq, len;
    }

    /**
     * ClosedownSource:
     * @CLOSEDOWN_LOCAL: Error detected locally, or connection forcefully closed
     * locally.
     * @CLOSEDOWN_REMOTE: RST segment received from the peer.
     *
     * Reasons for calling closedown().
     *
     * Since: 0.1.8
     */
    enum ClosedownSource{
      CLOSEDOWN_LOCAL,
      CLOSEDOWN_REMOTE
    }

    /**
     * PseudoTcpState:
     * @TCP_LISTEN: The socket's initial state. The socket isn't connected and is
     * listening for an incoming connection
     * @TCP_SYN_SENT: The socket has sent a connection request (SYN) packet and is
     * waiting for an answer
     * @TCP_SYN_RECEIVED: The socket has received a connection request (SYN) packet.
     * @TCP_ESTABLISHED: The socket is connected
     * @TCP_CLOSED: The socket has been closed
     * @TCP_FIN_WAIT_1: The socket has been closed locally but not remotely
     * (Since: 0.1.8)
     * @TCP_FIN_WAIT_2: The socket has been closed locally but not remotely
     * (Since: 0.1.8)
     * @TCP_CLOSING: The socket has been closed locally and remotely
     * (Since: 0.1.8)
     * @TCP_TIME_WAIT: The socket has been closed locally and remotely
     * (Since: 0.1.8)
     * @TCP_CLOSE_WAIT: The socket has been closed remotely but not locally
     * (Since: 0.1.8)
     * @TCP_LAST_ACK: The socket has been closed locally and remotely
     * (Since: 0.1.8)
     *
     * An enum representing the state of the #PseudoTcpSocket. These states
     * correspond to the TCP states in RFC 793.
     * <para> See also: #PseudoTcpSocket:state </para>
     *
     * Since: 0.0.11
     */
    enum PseudoTcpState{
      TCP_LISTEN,
      TCP_SYN_SENT,
      TCP_SYN_RECEIVED,
      TCP_ESTABLISHED,
      TCP_CLOSED,
      TCP_FIN_WAIT_1,
      TCP_FIN_WAIT_2,
      TCP_CLOSING,
      TCP_TIME_WAIT,
      TCP_CLOSE_WAIT,
      TCP_LAST_ACK,
    }

    /**
        * PseudoTcpCallbacks:
        * @user_data: A user defined pointer to be passed to the callbacks
        * @PseudoTcpOpened: The #PseudoTcpSocket is now connected
        * @PseudoTcpReadable: The socket is readable
        * @PseudoTcpWritable: The socket is writable
        * @PseudoTcpClosed: The socket was closed (both sides)
        * @WritePacket: This callback is called when the socket needs to send data.
        *
        * A structure containing callbacks functions that will be called by the
        * #PseudoTcpSocket when some events happen.
        * <para> See also: #PseudoTcpWriteResult </para>
        *
        * Since: 0.0.11
        */
    public class PseudoTcpCallbacks{
        /*gpointer user_data;
        void  (*PseudoTcpOpened) (PseudoTcpSocket *tcp, gpointer data);
        void  (*PseudoTcpReadable) (PseudoTcpSocket *tcp, gpointer data);
        void  (*PseudoTcpWritable) (PseudoTcpSocket *tcp, gpointer data);
        void  (*PseudoTcpClosed) (PseudoTcpSocket *tcp, guint32 error, gpointer data);
        PseudoTcpWriteResult (*WritePacket) (PseudoTcpSocket *tcp,
            const gchar * buffer, guint32 len, gpointer data);*/

        public delegate void Callback(PseudoTcpSocket tcp, object data);
        public delegate void ClosedCallback(PseudoTcpSocket tcp, uint error, object data);
        public delegate PseudoTcpWriteResult WritePacketCallback(PseudoTcpSocket tcp, byte[] buffer, uint len, object data);

        public object user_data;

        public Callback PseudoTcpOpened;

        public Callback PseudoTcpReadable;

        public Callback PseudoTcpWritable;

        public ClosedCallback PseudoTcpClosed;

        public WritePacketCallback WritePacket;
        // return PseudoTcpWriteResult.WR_SUCCESS;
    }

    class PseudoTcpSocketPrivate {
      internal PseudoTcpCallbacks callbacks;

      internal Shutdown shutdown;  /* only used if !support_fin_ack */
      internal gboolean shutdown_reads;
      internal gint error;

      // TCB data
      internal PseudoTcpState state;
      internal guint32 conv;
      internal gboolean bReadEnable, bWriteEnable, bOutgoing;
      internal guint32 last_traffic;

      // Incoming data
      internal List<RSegment> rlist;
      internal guint32 rbuf_len, rcv_nxt, rcv_wnd, lastrecv;
      internal guint8 rwnd_scale; // Window scale factor
      internal PseudoTcpFifo rbuf;
      internal guint32 rcv_fin;  /* sequence number of the received FIN octet, or 0 */

      // Outgoing data
      internal List<SSegment> slist;
      internal List<SSegment> unsent_slist;
      internal guint32 sbuf_len, snd_nxt, snd_wnd, lastsend;
      internal guint32 snd_una;  /* oldest unacknowledged sequence number */
      internal guint8 swnd_scale; // Window scale factor
      internal PseudoTcpFifo sbuf;

      // Maximum segment size, estimated protocol level, largest segment sent
      internal guint32 mss, msslevel, largest, mtu_advise;
      // Retransmit timer
      internal guint32 rto_base;

      // Timestamp tracking
      internal guint32 ts_recent, ts_lastack;

      // Round-trip calculation
      internal guint32 rx_rttvar, rx_srtt, rx_rto;

      // Congestion avoidance, Fast retransmit/recovery, Delayed ACKs
      internal guint32 ssthresh, cwnd;
      internal guint8 dup_acks;
      internal guint32 recover;
      internal gboolean fast_recovery;
      internal guint32 t_ack;  /* time a delayed ack was scheduled; 0 if no acks scheduled */
      internal guint32 last_acked_ts;

      internal gboolean use_nagling;
      internal guint32 ack_delay;

      // This is used by unit tests to test backward compatibility of
      // PseudoTcp implementations that don't support window scaling.
      internal gboolean support_wnd_scale;

      /* Current time. Typically only used for testing, when non-zero. When zero,
       * the system monotonic clock is used. Units: monotonic milliseconds. */
      internal guint32 current_time;

      /* This is used by compatible implementations (with the TCP_OPT_FIN_ACK
       * option) to enable correct FIN-ACK connection termination. Defaults to
       * true unless no compatible option is received. */
      internal gboolean support_fin_ack;
    }

    static bool LARGER(uint a, uint b) { return (((a) - (b) - 1) < (uint.MaxValue >> 1)); }

    static bool LARGER_OR_EQUAL(uint a, uint b) { return (((a) - (b)) < (uint.MaxValue >> 1)); }

    static bool SMALLER(uint a, uint b) { return LARGER (b,a); }
    static bool SMALLER_OR_EQUAL(uint a, uint b) { return LARGER_OR_EQUAL ((b),(a)); }

    /* properties */
    enum Props
    {
      PROP_CONVERSATION = 1,
      PROP_CALLBACKS,
      PROP_STATE,
      PROP_ACK_DELAY,
      PROP_NO_DELAY,
      PROP_RCV_BUF,
      PROP_SND_BUF,
      PROP_SUPPORT_FIN_ACK,
      LAST_PROPERTY
    };

/*
    static void pseudo_tcp_socket_get_property (GObject *object, guint property_id,
        GValue *value,  GParamSpec *pspec);
    static void pseudo_tcp_socket_set_property (GObject *object, guint property_id,
        const GValue *value, GParamSpec *pspec);
    static void pseudo_tcp_socket_finalize (GObject *object);


    static void queue_connect_message (PseudoTcpSocket *self);
    static uint queue (PseudoTcpSocket *self, const gchar *data,
        uint len, TcpFlags flags);
    static PseudoTcpWriteResult packet(PseudoTcpSocket *self, uint seq,
        TcpFlags flags, uint offset, uint len, uint now);
    static bool parse (PseudoTcpSocket *self,
        const byte *_header_buf, gsize header_buf_len,
        const byte *data_buf, gsize data_buf_len);
    static bool process(PseudoTcpSocket *self, Segment *seg);
    static int transmit(PseudoTcpSocket *self, SSegment *sseg, uint now);
    static void attempt_send(PseudoTcpSocket *self, SendFlags sflags);
    static void closedown (PseudoTcpSocket *self, uint err,
        ClosedownSource source);
    static void adjustMTU(PseudoTcpSocket *self);
    static void parse_options (PseudoTcpSocket *self, const byte *data,
        uint len);
    static void resize_send_buffer (PseudoTcpSocket *self, uint new_size);
    static void resize_receive_buffer (PseudoTcpSocket *self, uint new_size);
    static void set_state (PseudoTcpSocket *self, PseudoTcpState new_state);
    static void set_state_established (PseudoTcpSocket *self);
    static void set_state_closed (PseudoTcpSocket *self, uint err);

    static const string pseudo_tcp_state_get_name (PseudoTcpState state);
    static bool pseudo_tcp_state_has_sent_fin (PseudoTcpState state);
    static bool pseudo_tcp_state_has_received_fin (PseudoTcpState state);
    static bool pseudo_tcp_state_has_received_fin_ack (PseudoTcpState state);*/


    /**
     * PseudoTcpDebugLevel:
     * @PSEUDO_TCP_DEBUG_NONE: Disable debug messages
     * @PSEUDO_TCP_DEBUG_NORMAL: Enable basic debug messages
     * @PSEUDO_TCP_DEBUG_VERBOSE: Enable verbose debug messages
     *
     * Valid values of debug levels to be set.
     *
     * Since: 0.0.11
     */
    enum PseudoTcpDebugLevel {
      PSEUDO_TCP_DEBUG_NONE = 0,
      PSEUDO_TCP_DEBUG_NORMAL,
      PSEUDO_TCP_DEBUG_VERBOSE
    }

    // The following logging is for detailed (packet-level) pseudotcp analysis only.
    static PseudoTcpDebugLevel debug_level = PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NONE;

    static void DEBUG(
        PseudoTcpSocket self,
        PseudoTcpDebugLevel level, string fmt, params object[] args)
    {
      if (debug_level >= level)
        Console.WriteLine (level == PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL ? "libnice-pseudotcp" : "libnice-pseudotcp-verbose" +
            /*G_LOG_LEVEL_DEBUG,*/ 
            string.Format("PseudoTcpSocket {0} {1}",
                self, pseudo_tcp_state_get_name (self.priv.state)) + string.Format(fmt, args));
    }

    void
    pseudo_tcp_set_debug_level (PseudoTcpDebugLevel level)
    {
      debug_level = level;
    }

    static guint32
    get_current_time (PseudoTcpSocket socket)
    {
      if (/*G_UNLIKELY*/ (socket.priv.current_time != 0))
        return socket.priv.current_time;

      return g_get_monotonic_time () /*/ 1000*/;
    }

    static uint g_get_monotonic_time()
    {
        // probably use StopWatch or something similar
        return (uint) Environment.TickCount;
    }

    void
    pseudo_tcp_socket_set_time(PseudoTcpSocket self, guint32 current_time)
    {
      self.priv.current_time = current_time;
    }

    static void
    pseudo_tcp_socket_class_init (PseudoTcpSocketClass cls)
    {
      /*GObjectClass *object_class = G_OBJECT_CLASS (cls);

      object_class.get_property = pseudo_tcp_socket_get_property;
      object_class.set_property = pseudo_tcp_socket_set_property;
      object_class.finalize = pseudo_tcp_socket_finalize;

      g_object_class_install_property (object_class, PROP_CONVERSATION,
          g_param_spec_uint ("conversation", "TCP Conversation ID",
              "The TCP Conversation ID",
              0, G_MAXUINT32, 0,
              G_PARAM_CONSTRUCT_ONLY | G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

      g_object_class_install_property (object_class, PROP_CALLBACKS,
          g_param_spec_pointer ("callbacks", "PseudoTcp socket callbacks",
              "Structure with the callbacks to call when PseudoTcp events happen",
              G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

      g_object_class_install_property (object_class, PROP_STATE,
          g_param_spec_uint ("state", "PseudoTcp State",
              "The current state (enum PseudoTcpState) of the PseudoTcp socket",
              TCP_LISTEN, TCP_CLOSED, TCP_LISTEN,
              G_PARAM_READABLE | G_PARAM_STATIC_STRINGS));

      g_object_class_install_property (object_class, PROP_ACK_DELAY,
          g_param_spec_uint ("ack-delay", "ACK Delay",
              "Delayed ACK timeout (in milliseconds)",
              0, G_MAXUINT, DEFAULT_ACK_DELAY,
              G_PARAM_READWRITE| G_PARAM_STATIC_STRINGS));

      g_object_class_install_property (object_class, PROP_NO_DELAY,
          g_param_spec_boolean ("no-delay", "No Delay",
              "Disable the Nagle algorithm (like the TCP_NODELAY option)",
              DEFAULT_NO_DELAY,
              G_PARAM_READWRITE| G_PARAM_STATIC_STRINGS));

      g_object_class_install_property (object_class, PROP_RCV_BUF,
          g_param_spec_uint ("rcv-buf", "Receive Buffer",
              "Receive Buffer size",
              1, G_MAXUINT, DEFAULT_RCV_BUF_SIZE,
              G_PARAM_READWRITE| G_PARAM_STATIC_STRINGS));

      g_object_class_install_property (object_class, PROP_SND_BUF,
          g_param_spec_uint ("snd-buf", "Send Buffer",
              "Send Buffer size",
              1, G_MAXUINT, DEFAULT_SND_BUF_SIZE,
              G_PARAM_READWRITE| G_PARAM_STATIC_STRINGS));*/

      /**
       * PseudoTcpSocket:support-fin-ack:
       *
       * Whether to support the FIN–ACK extension to the pseudo-TCP protocol for
       * this socket. The extension is only compatible with other libnice pseudo-TCP
       * stacks, and not with Jingle pseudo-TCP stacks. If enabled, support is
       * negotiatied on connection setup, so it is safe for a #PseudoTcpSocket with
       * support enabled to be used with one with it disabled, or with a Jingle
       * pseudo-TCP socket which doesn’t support it at all.
       *
       * Support is enabled by default.
       *
       * Since: 0.1.8
       */
      /*g_object_class_install_property (object_class, PROP_SUPPORT_FIN_ACK,
          g_param_spec_boolean ("support-fin-ack", "Support FIN–ACK",
              "Whether to enable the optional FIN–ACK support.",
              true,
              G_PARAM_READWRITE | G_PARAM_CONSTRUCT_ONLY | G_PARAM_STATIC_STRINGS));*/
    }


    /*static void
    pseudo_tcp_socket_get_property (GObject *object,
                                      guint property_id,
                                      GValue *value,
                                      GParamSpec *pspec)
    {
      PseudoTcpSocket *self = PSEUDO_TCP_SOCKET (object);

      switch (property_id) {
        case PROP_CONVERSATION:
          g_value_set_uint (value, self.priv.conv);
          break;
        case PROP_CALLBACKS:
          g_value_set_pointer (value, (gpointer) &self.priv.callbacks);
          break;
        case PROP_STATE:
          g_value_set_uint (value, self.priv.state);
          break;
        case PROP_ACK_DELAY:
          g_value_set_uint (value, self.priv.ack_delay);
          break;
        case PROP_NO_DELAY:
          g_value_set_boolean (value, !self.priv.use_nagling);
          break;
        case PROP_RCV_BUF:
          g_value_set_uint (value, self.priv.rbuf_len);
          break;
        case PROP_SND_BUF:
          g_value_set_uint (value, self.priv.sbuf_len);
          break;
        case PROP_SUPPORT_FIN_ACK:
          g_value_set_boolean (value, self.priv.support_fin_ack);
          break;
        default:
          G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
          break;
      }
    }*/

    /*static void
    pseudo_tcp_socket_set_property (GObject *object,
                                      guint property_id,
                                      const GValue *value,
                                      GParamSpec *pspec)
    {
      PseudoTcpSocket *self = PSEUDO_TCP_SOCKET (object);

      switch (property_id) {
        case PROP_CONVERSATION:
          self.priv.conv = g_value_get_uint (value);
          break;
        case PROP_CALLBACKS:
          {
            PseudoTcpCallbacks *c = g_value_get_pointer (value);
            self.priv.callbacks = *c;
          }
          break;
        case PROP_ACK_DELAY:
          self.priv.ack_delay = g_value_get_uint (value);
          break;
        case PROP_NO_DELAY:
          self.priv.use_nagling = !g_value_get_boolean (value);
          break;
        case PROP_RCV_BUF:
          g_return_if_fail (self.priv.state == TCP_LISTEN);
          resize_receive_buffer (self, g_value_get_uint (value));
          break;
        case PROP_SND_BUF:
          g_return_if_fail (self.priv.state == TCP_LISTEN);
          resize_send_buffer (self, g_value_get_uint (value));
          break;
        case PROP_SUPPORT_FIN_ACK:
          self.priv.support_fin_ack = g_value_get_boolean (value);
          break;
        default:
          G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
          break;
      }
    }*/

    /*static void
    pseudo_tcp_socket_finalize (GObject *object)
    {
      PseudoTcpSocket *self = PSEUDO_TCP_SOCKET (object);
      PseudoTcpSocketPrivate *priv = self.priv;
      GList *i;
      SSegment *sseg;

      if (priv == NULL)
        return;

      while ((sseg = g_queue_pop_head (&priv.slist)))
        g_slice_free (SSegment, sseg);
      g_queue_clear (&priv.unsent_slist);
      for (i = priv.rlist; i; i = i.next) {
        RSegment *rseg = i.data;
        g_slice_free (RSegment, rseg);
      }
      g_list_free (priv.rlist);
      priv.rlist = NULL;

      pseudo_tcp_fifo_clear (&priv.rbuf);
      pseudo_tcp_fifo_clear (&priv.sbuf);

      g_free (priv);
      self.priv = NULL;

      if (G_OBJECT_CLASS (pseudo_tcp_socket_parent_class).finalize)
        G_OBJECT_CLASS (pseudo_tcp_socket_parent_class).finalize (object);
    }*/


    static void
    pseudo_tcp_socket_init (PseudoTcpSocket obj)
    {
      /* Use g_new0, and do not use g_object_set_private because the size of
       * our private data is too big (150KB+) and the g_slice_allow cannot allocate
       * it. So we handle the private ourselves */
      PseudoTcpSocketPrivate priv = new PseudoTcpSocketPrivate();

      obj.priv = priv;

      priv.shutdown = Shutdown.SD_NONE;
      priv.error = 0;

      priv.rbuf_len = DEFAULT_RCV_BUF_SIZE;
      pseudo_tcp_fifo_init (priv.rbuf, priv.rbuf_len);
      priv.sbuf_len = DEFAULT_SND_BUF_SIZE;
      pseudo_tcp_fifo_init (priv.sbuf, priv.sbuf_len);

      priv.state = PseudoTcpState.TCP_LISTEN;
      priv.conv = 0;

      priv.slist = new List<SSegment>();
      priv.unsent_slist = new List<SSegment>();

      priv.rcv_wnd = priv.rbuf_len;
      priv.rwnd_scale = priv.swnd_scale = 0;
      priv.snd_nxt = 0;
      priv.snd_wnd = 1;
      priv.snd_una = priv.rcv_nxt = 0;
      priv.bReadEnable = true;
      priv.bWriteEnable = false;
      priv.rcv_fin = 0;

      priv.t_ack = 0;

      priv.msslevel = 0;
      priv.largest = 0;
      priv.mss = MIN_PACKET - PACKET_OVERHEAD;
      priv.mtu_advise = DEF_MTU;

      priv.rto_base = 0;

      priv.cwnd = 2 * priv.mss;
      priv.ssthresh = priv.rbuf_len;
      priv.lastrecv = priv.lastsend = priv.last_traffic = 0;
      priv.bOutgoing = false;

      priv.dup_acks = 0;
      priv.recover = 0;
      priv.last_acked_ts = 0;

      priv.ts_recent = priv.ts_lastack = 0;

      priv.rx_rto = DEF_RTO;
      priv.rx_srtt = priv.rx_rttvar = 0;

      priv.ack_delay = DEFAULT_ACK_DELAY;
      priv.use_nagling = !DEFAULT_NO_DELAY;

      priv.support_wnd_scale = true;
      priv.support_fin_ack = true;
    }

    static PseudoTcpSocket pseudo_tcp_socket_new (uint conversation,
        PseudoTcpCallbacks callbacks)
    {
        PseudoTcpSocket result = new PseudoTcpSocket();
        result.priv.callbacks = callbacks;
        return result;
    }

    static void
    queue_connect_message (PseudoTcpSocket self)
    {
      PseudoTcpSocketPrivate priv = self.priv;
      guint8[] buf = new guint8[8];
      gsize size = 0;

      buf[size++] = CTL_CONNECT;

      if (priv.support_wnd_scale) {
        buf[size++] = (byte) TcpOption.TCP_OPT_WND_SCALE;
        buf[size++] = 1;
        buf[size++] = priv.rwnd_scale;
      }

      if (priv.support_fin_ack) {
        buf[size++] = (byte) TcpOption.TCP_OPT_FIN_ACK;
        buf[size++] = 1;  /* option length; zero is invalid (RFC 1122, §4.2.2.5) */
        buf[size++] = 0;  /* currently unused */
      }

      priv.snd_wnd = size;

      queue (self, buf, size, TcpFlags.FLAG_CTL);
    }

    static void
    queue_fin_message (PseudoTcpSocket self)
    {
      g_assert (self.priv.support_fin_ack);

      /* FIN segments are always zero-length. */
      queue (self, null, 0, TcpFlags.FLAG_FIN);
    }

    static void
    queue_rst_message (PseudoTcpSocket self)
    {
      g_assert (self.priv.support_fin_ack);

      /* RST segments are always zero-length. */
      queue (self, null, 0, TcpFlags.FLAG_RST);
    }

    bool
    pseudo_tcp_socket_connect(PseudoTcpSocket self)
    {
      PseudoTcpSocketPrivate priv = self.priv;

      if (priv.state != PseudoTcpState.TCP_LISTEN) {
        priv.error = EINVAL;
        return false;
      }

      set_state (self, PseudoTcpState.TCP_SYN_SENT);

      queue_connect_message (self);
      attempt_send(self, SendFlags.sfNone);

      return true;
    }

    void
    pseudo_tcp_socket_notify_mtu(PseudoTcpSocket self, guint16 mtu)
    {
      PseudoTcpSocketPrivate priv = self.priv;
      priv.mtu_advise = mtu;
      if (priv.state == PseudoTcpState.TCP_ESTABLISHED) {
        adjustMTU(self);
      }
    }

    static int g_queue_get_length(List<SSegment> queue)
    {
        return queue.Count;
    }

    static SSegment g_queue_peek_tail(List<SSegment> list)
    {
        if (list.Count == 0)
            return null;

        return list[list.Count - 1];
    }

    static void g_assert_not_reached ()
    {
        throw new Exception("g_assert_not_reached");
    }

    static SSegment g_queue_peek_head(List<SSegment> queue)
    {
        if (queue.Count == 0)
            return null;

        return queue[0];
    }

    static void g_queue_push_tail (List<SSegment> queue, SSegment segment)
    {
        queue.Add(segment);
    }

    static void g_queue_insert_after(List<SSegment> list, int pos, SSegment segment)
    {
        if (pos >= list.Count -1)
        {
            list.Add(segment);

            return;
        }

        list.Insert(pos + 1, segment);
    }

    static int g_queue_find(List<SSegment> list, SSegment toFind)
    {
        return list.IndexOf(toFind);
    }

    static SSegment g_queue_pop_head(List<SSegment> queue)
    {
        if (queue.Count == 0)
            return null;

        SSegment result = queue[0];

        queue.RemoveAt(0);

        return result;
    }

    void
    pseudo_tcp_socket_notify_clock(PseudoTcpSocket self)
    {
      PseudoTcpSocketPrivate priv = self.priv;
      guint32 now = get_current_time(self);

      if (priv.state == PseudoTcpState.TCP_CLOSED)
        return;

      /* If in the TIME-WAIT state, any delayed segments have passed and the
       * connection can be considered closed from both ends.
       * FIXME: This should probably actually compare a timestamp before
       * operating. */
      if (priv.support_fin_ack && priv.state == PseudoTcpState.TCP_TIME_WAIT) {
        DEBUG(self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL,
            "Notified clock in TIME-WAIT state; closing connection.");
        set_state_closed (self, 0);
      }

      /* If in the LAST-ACK state, resend the FIN because it hasn’t been ACKed yet.
       * FIXME: This should probably actually compare a timestamp before
       * operating. */
      if (priv.support_fin_ack && priv.state == PseudoTcpState.TCP_LAST_ACK) {
        DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL,
            "Notified clock in LAST-ACK state; resending FIN segment.");
        queue_fin_message (self);
        attempt_send (self, SendFlags.sfFin);
      }

      // Check if it's time to retransmit a segment
      if ((priv.rto_base != 0) &&
          (time_diff(priv.rto_base + priv.rx_rto, now) <= 0)) {
        if (g_queue_get_length (priv.slist) == 0) {
          g_assert_not_reached ();
        } else {
          // Note: (priv.slist.front().xmit == 0)) {
          // retransmit segments
          guint32 nInFlight;
          guint32 rto_limit;
          int transmit_status;

          DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL,
              "timeout retransmit (rto: {0}) (rto_base: {1}) (now: {2}) (dup_acks: {3})",
              priv.rx_rto, priv.rto_base, now, (uint) priv.dup_acks);

          transmit_status = transmit(self, g_queue_peek_head (priv.slist), now);
          if (transmit_status != 0) {
            DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL,
                "Error transmitting segment. Closing down.");
            closedown (self, (uint) transmit_status, ClosedownSource.CLOSEDOWN_LOCAL);
            return;
          }

          nInFlight = priv.snd_nxt - priv.snd_una;
          priv.ssthresh = Math.Max(nInFlight / 2, 2 * priv.mss);
          DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL,
              "ssthresh: {0} = (nInFlight: {1} / 2) + 2 * mss: {2}",
              priv.ssthresh, nInFlight, priv.mss);

          //LOG(LS_INFO) << "priv.ssthresh: " << priv.ssthresh << "  nInFlight: " << nInFlight << "  priv.mss: " << priv.mss;
          priv.cwnd = priv.mss;

          // Back off retransmit timer.  Note: the limit is lower when connecting.
          rto_limit = (uint) ((priv.state < PseudoTcpState.TCP_ESTABLISHED) ? DEF_RTO : MAX_RTO);
          priv.rx_rto = Math.Min(rto_limit, priv.rx_rto * 2);
          priv.rto_base = now;

          priv.recover = priv.snd_nxt;
          if (priv.dup_acks >= 3) {
            priv.dup_acks = 0;
            priv.fast_recovery = false;
            DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "exit recovery on timeout");
          }
        }
      }

      // Check if it's time to probe closed windows
      if ((priv.snd_wnd == 0)
            && (time_diff(priv.lastsend + priv.rx_rto, now) <= 0)) {
        if (time_diff(now, priv.lastrecv) >= 15000) {
          DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "Receive window closed. Closing down.");
          closedown (self, ECONNABORTED, ClosedownSource.CLOSEDOWN_LOCAL);
          return;
        }

        // probe the window
        packet(self, priv.snd_nxt - 1, 0, 0, 0, now);
        priv.lastsend = now;

        // back off retransmit timer
        priv.rx_rto = Math.Min(MAX_RTO, priv.rx_rto * 2);
      }

      // Check if it's time to send delayed acks
      if ((priv.t_ack != 0) && (time_diff(priv.t_ack + priv.ack_delay, now) <= 0)) {
        packet(self, priv.snd_nxt, 0, 0, 0, now);
      }

    }

    gboolean
    pseudo_tcp_socket_notify_packet(PseudoTcpSocket self,
        byte[] buffer, guint32 len)
    {
      gboolean retval;

      if (len > MAX_PACKET) {
        //LOG_F(WARNING) << "packet too large";
        self.priv.error = EMSGSIZE;
        return false;
      } else if (len < HEADER_SIZE) {
        //LOG_F(WARNING) << "packet too small";
        self.priv.error = EINVAL;
        return false;
      }

      /* Hold a reference to the PseudoTcpSocket during parsing, since it may be
       * closed from within a callback. */
      //g_object_ref (self);

#warning -> copying buffers is not optimal!!! We should use a stream or pointers
      byte[] data_buf = new byte[len - HEADER_SIZE];
      Buffer.BlockCopy(buffer, HEADER_SIZE, data_buf, 0, data_buf.Length);

      retval = parse (self, buffer, HEADER_SIZE,
          data_buf, len - HEADER_SIZE);
      //g_object_unref (self);

      return retval;
    }

    /* Assume there are two buffers in the given #NiceInputMessage: a 24-byte one
     * containing the header, and a bigger one for the data. */
    /*bool
    pseudo_tcp_socket_notify_message (PseudoTcpSocket self,
        NiceInputMessage *message)
    {
      bool retval;

      g_assert_cmpuint (message.n_buffers, >, 0);

      if (message.n_buffers == 1)
        return pseudo_tcp_socket_notify_packet (self, message.buffers[0].buffer,
            message.buffers[0].size);

      g_assert_cmpuint (message.n_buffers, ==, 2);
      g_assert_cmpuint (message.buffers[0].size, ==, HEADER_SIZE);

      if (message.length > MAX_PACKET) {
        //LOG_F(WARNING) << "packet too large";
        return false;
      } else if (message.length < HEADER_SIZE) {
        //LOG_F(WARNING) << "packet too small";
        return false;
      }

      // Hold a reference to the PseudoTcpSocket during parsing, since it may be
      // closed from within a callback.
      g_object_ref (self);
      retval = parse (self, message.buffers[0].buffer, message.buffers[0].size,
          message.buffers[1].buffer, message.length - message.buffers[0].size);
      g_object_unref (self);

      return retval;
    }*/

    gboolean
    pseudo_tcp_socket_get_next_clock(PseudoTcpSocket self, ref guint64 timeout)
    {
      PseudoTcpSocketPrivate priv = self.priv;
      guint32 now = get_current_time(self);
      gsize snd_buffered;
      guint32 closed_timeout;

      if (priv.shutdown == Shutdown.SD_FORCEFUL) {
        if (priv.support_fin_ack) {
          DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL,
              "‘Forceful’ shutdown used when FIN-ACK support is enabled");
        }

        /* Transition to the CLOSED state. */
        closedown (self, 0, ClosedownSource.CLOSEDOWN_REMOTE);

        return false;
      }

      snd_buffered = pseudo_tcp_fifo_get_buffered (priv.sbuf);
      if ((priv.shutdown == Shutdown.SD_GRACEFUL)
          && ((priv.state != PseudoTcpState.TCP_ESTABLISHED)
              || ((snd_buffered == 0) && (priv.t_ack == 0)))) {
        if (priv.support_fin_ack) {
          DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL,
              "‘Graceful’ shutdown used when FIN-ACK support is enabled");
        }

        /* Transition to the CLOSED state. */
        closedown (self, 0, ClosedownSource.CLOSEDOWN_REMOTE);

        return false;
      }

      /* FIN-ACK support. The timeout for closing the socket if nothing is received
       * varies depending on whether the socket is waiting in the TIME-WAIT state
       * for delayed segments to pass.
       *
       * See: http://vincent.bernat.im/en/blog/2014-tcp-time-wait-state-linux.html
       */
      closed_timeout = CLOSED_TIMEOUT;
      if (priv.support_fin_ack && priv.state == PseudoTcpState.TCP_TIME_WAIT)
        closed_timeout = TIME_WAIT_TIMEOUT;

      if (priv.support_fin_ack && priv.state == PseudoTcpState.TCP_CLOSED) {
        return false;
      }

      if (timeout == 0 || timeout < now)
        timeout = now + closed_timeout;

      if (priv.support_fin_ack && priv.state == PseudoTcpState.TCP_TIME_WAIT) {
        timeout = Math.Min (timeout, now + TIME_WAIT_TIMEOUT);
        return true;
      }

      if (priv.state == PseudoTcpState.TCP_CLOSED && !priv.support_fin_ack) {
        timeout = Math.Min (timeout, now + CLOSED_TIMEOUT);
        return true;
      }

      timeout = Math.Min (timeout, now + DEFAULT_TIMEOUT);

      if (priv.t_ack != 0) {
        timeout = Math.Min(timeout, priv.t_ack + priv.ack_delay);
      }
      if (priv.rto_base != 0) {
        timeout = Math.Min(timeout, priv.rto_base + priv.rx_rto);
      }
      if (priv.snd_wnd == 0) {
        timeout = Math.Min(timeout, priv.lastsend + priv.rx_rto);
      }

      return true;
    }


    gint
    pseudo_tcp_socket_recv(PseudoTcpSocket self, byte[] buffer, size_t len)
    {
      PseudoTcpSocketPrivate priv = self.priv;
      gsize bytesread;
      gsize available_space;

      /* Received a FIN from the peer, so return 0. RFC 793, §3.5, Case 2. */
      if (priv.support_fin_ack && priv.shutdown_reads) {
        return 0;
      }

      /* Return 0 if FIN-ACK is not supported but the socket has been closed. */
      if (!priv.support_fin_ack && pseudo_tcp_socket_is_closed (self)) {
        return 0;
      }

      /* Return ENOTCONN if FIN-ACK is not supported and the connection is not
       * ESTABLISHED. */
      if (!priv.support_fin_ack && priv.state != PseudoTcpState.TCP_ESTABLISHED) {
        priv.error = ENOTCONN;
        return -1;
      }

      if (len == 0)
        return 0;

      bytesread = pseudo_tcp_fifo_read (priv.rbuf, buffer, len);

     // If there's no data in |m_rbuf|.
      if (bytesread == 0 &&
          !(pseudo_tcp_state_has_received_fin (priv.state) ||
            pseudo_tcp_state_has_received_fin_ack (priv.state))) {
        priv.bReadEnable = true;
        priv.error = EWOULDBLOCK;
        return -1;
      }

      available_space = pseudo_tcp_fifo_get_write_remaining (priv.rbuf);

      if (available_space - priv.rcv_wnd >=
          Math.Min (priv.rbuf_len / 2, priv.mss)) {
        // !?! Not sure about this was closed business
        bool bWasClosed = (priv.rcv_wnd == 0);

        priv.rcv_wnd = (uint) available_space;

        if (bWasClosed) {
          attempt_send(self, SendFlags.sfImmediateAck);
        }
      }

#warning this cast was not here in C code
      return (gint) bytesread;
    }

    gint
    pseudo_tcp_socket_send(PseudoTcpSocket self, byte[] buffer, guint32 len)
    {
      PseudoTcpSocketPrivate priv = self.priv;
      gint written;
      gsize available_space;

      if (priv.state != PseudoTcpState.TCP_ESTABLISHED) {
        priv.error = pseudo_tcp_state_has_sent_fin (priv.state) ? EPIPE : ENOTCONN;
        return -1;
      }

      available_space = pseudo_tcp_fifo_get_write_remaining (priv.sbuf);

      if (available_space == 0) {
        priv.bWriteEnable = true;
        priv.error = EWOULDBLOCK;
        return -1;
      }

#warning this cast was not here in C code (gint)
      written = (gint) queue (self, buffer, len, TcpFlags.FLAG_NONE);
      attempt_send(self, SendFlags.sfNone);

      if (written > 0 && (uint)written < len) {
        priv.bWriteEnable = true;
      }

      return written;
    }

    void
    pseudo_tcp_socket_close(PseudoTcpSocket self, gboolean force)
    {
      PseudoTcpSocketPrivate priv = self.priv;

      DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "Closing socket {0} {1}",
          force ? "forcefully" : "gracefully");

      /* Forced closure by sending an RST segment. RFC 1122, §4.2.2.13. */
      if (force && priv.state != PseudoTcpState.TCP_CLOSED) {
        closedown (self, ECONNABORTED, ClosedownSource.CLOSEDOWN_LOCAL);
        return;
      }

      /* Fall back to shutdown(). */
      pseudo_tcp_socket_shutdown (self, PseudoTcpShutdown.PSEUDO_TCP_SHUTDOWN_RDWR);
    }

    void
    pseudo_tcp_socket_shutdown (PseudoTcpSocket self, PseudoTcpShutdown how)
    {
      PseudoTcpSocketPrivate priv = self.priv;

      DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "Shutting down socket %p: %u", self, how);

      /* FIN-ACK--only stuff below here. */
      if (!priv.support_fin_ack) {
        if (priv.shutdown == Shutdown.SD_NONE)
          priv.shutdown = Shutdown.SD_GRACEFUL;
        return;
      }

      /* What needs shutting down? */
      switch (how) {
      case PseudoTcpShutdown.PSEUDO_TCP_SHUTDOWN_RD:
      case PseudoTcpShutdown.PSEUDO_TCP_SHUTDOWN_RDWR:
        priv.shutdown_reads = true;
        break;
      case PseudoTcpShutdown.PSEUDO_TCP_SHUTDOWN_WR:
        /* Handled below. */
        break;
      default:
        DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "Invalid shutdown method: {0}.", how);
        break;
      }

      if (how == PseudoTcpShutdown.PSEUDO_TCP_SHUTDOWN_RD) {
        return;
      }

      /* Unforced write closure. */
      switch (priv.state) {
      case PseudoTcpState.TCP_LISTEN:
      case PseudoTcpState.TCP_SYN_SENT:
        /* Just abort the connection without completing the handshake. */
        set_state_closed (self, 0);
        break;
      case PseudoTcpState.TCP_SYN_RECEIVED:
      case PseudoTcpState.TCP_ESTABLISHED:
        /* Local user initiating the close: RFC 793, §3.5, Cases 1 and 3.
         * If there is pending receive data, send RST instead of FIN;
         * see RFC 1122, §4.2.2.13. */
        if (pseudo_tcp_socket_get_available_bytes (self) > 0) {
          closedown (self, ECONNABORTED, ClosedownSource.CLOSEDOWN_LOCAL);
        } else {
          queue_fin_message (self);
          attempt_send (self, SendFlags.sfFin);
          set_state (self, PseudoTcpState.TCP_FIN_WAIT_1);
        }
        break;
      case PseudoTcpState.TCP_CLOSE_WAIT:
        /* Remote user initiating the close: RFC 793, §3.5, Case 2.
         * We’ve previously received a FIN from the peer; now the user is closing
         * the local end of the connection. */
        queue_fin_message (self);
        attempt_send (self, SendFlags.sfFin);
        set_state (self, PseudoTcpState.TCP_LAST_ACK);
        break;
      case PseudoTcpState.TCP_CLOSING:
      case PseudoTcpState.TCP_CLOSED:
        /* Already closed on both sides. */
        break;
      case PseudoTcpState.TCP_FIN_WAIT_1:
      case PseudoTcpState.TCP_FIN_WAIT_2:
      case PseudoTcpState.TCP_TIME_WAIT:
      case PseudoTcpState.TCP_LAST_ACK:
        /* Already closed locally. */
        break;
      default:
        /* Do nothing. */
        break;
      }
    }

/*    int
    pseudo_tcp_socket_get_error(PseudoTcpSocket self)
    {
      PseudoTcpSocketPrivate priv = self.priv;
      return priv.error;
    }*/

    //
    // Internal Implementation
    //

    static guint32
    queue(PseudoTcpSocket self, /*const gchar * */ byte[] data, guint32 len, TcpFlags flags)
    {
      PseudoTcpSocketPrivate priv = self.priv;
      gsize available_space;

      available_space = pseudo_tcp_fifo_get_write_remaining (priv.sbuf);
      if (len > available_space) {
        g_assert (flags == TcpFlags.FLAG_NONE);
        len = (uint) available_space;
      }

      // We can concatenate data if the last segment is the same type
      // (control v. regular data), and has not been transmitted yet
      if ((g_queue_get_length (priv.slist) != 0) &&
          ((g_queue_peek_tail (priv.slist)).flags == flags) &&
          ((g_queue_peek_tail (priv.slist)).xmit == 0)) {
        (g_queue_peek_tail (priv.slist)).len += len;
      } else {
        SSegment sseg = new SSegment();
        gsize snd_buffered = pseudo_tcp_fifo_get_buffered (priv.sbuf);

        sseg.seq = (size_t) (priv.snd_una + snd_buffered);
        sseg.len = len;
        sseg.flags = flags;
        g_queue_push_tail (priv.slist, sseg);
        g_queue_push_tail (priv.unsent_slist, sseg);
      }

      //LOG(LS_INFO) << "PseudoTcp::queue - priv.slen = " << priv.slen;
      return pseudo_tcp_fifo_write (priv.sbuf, data, len);
    }

    // Creates a packet and submits it to the network. This method can either
    // send payload or just an ACK packet.
    //
    // |seq| is the sequence number of this packet.
    // |flags| is the flags for sending this packet.
    // |offset| is the offset to read from |m_sbuf|.
    // |len| is the number of bytes to read from |m_sbuf| as payload. If this
    // value is 0 then this is an ACK packet, otherwise this packet has payload.

    static PseudoTcpWriteResult
    packet(PseudoTcpSocket self, guint32 seq, TcpFlags flags,
        guint32 offset, guint32 len, guint32 now)
    {
      PseudoTcpSocketPrivate priv = self.priv;
      /*union {
        byte u8[MAX_PACKET];
        guint16 u16[MAX_PACKET / 2];
        guint32 u32[MAX_PACKET / 4];
      } buffer;*/

      byte[] buffer = new byte[MAX_PACKET];

      PseudoTcpWriteResult wres = PseudoTcpWriteResult.WR_SUCCESS;

      g_assert(HEADER_SIZE + len <= MAX_PACKET);

      /* *buffer.u32 = htonl(priv.conv);
      *(buffer.u32 + 1) = htonl(seq);
      *(buffer.u32 + 2) = htonl(priv.rcv_nxt);

      buffer.u8[12] = 0;
      buffer.u8[13] = flags;
      *(buffer.u16 + 7) = htons((guint16)(priv.rcv_wnd >> priv.rwnd_scale));

      // Timestamp computations
      *(buffer.u32 + 4) = htonl(now);
      *(buffer.u32 + 5) = htonl(priv.ts_recent);*/

      // this can be rewritten for speed using unsafe and fixed and pointers
      using (System.IO.MemoryStream st = new System.IO.MemoryStream(buffer))
      using (System.IO.BinaryWriter writer = new System.IO.BinaryWriter(st))
      {
          writer.Write((uint)priv.conv);
          writer.Write((uint)seq);
          writer.Write((uint)priv.rcv_nxt);

          writer.Write((byte)0);
          writer.Write((byte)flags);
          writer.Write((ushort)(priv.rcv_wnd >> priv.rwnd_scale));

          // Timestamp computations
          writer.Write((uint)now);
          writer.Write((uint)(priv.ts_recent));
      }

      priv.ts_lastack = priv.rcv_nxt;

      if (len != 0) {
        gsize bytes_read;

        bytes_read = pseudo_tcp_fifo_read_offset (priv.sbuf, buffer, HEADER_SIZE,
            len, offset);
        g_assert (bytes_read == len);
      }

      DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_VERBOSE, 
          "Sending <CONV={0}><FLG={1}><SEQ={2}:{3}><ACK={4}><WND={5}><TS={6}><TSR={7}><LEN={8}>",
          priv.conv, flags, seq, seq + len, priv.rcv_nxt, priv.rcv_wnd,
          now % 10000, priv.ts_recent % 10000, len);

      wres = priv.callbacks.WritePacket(self, buffer, len + HEADER_SIZE,
                                         priv.callbacks.user_data);
      /* Note: When len is 0, this is an ACK packet.  We don't read the
         return value for those, and thus we won't retry.  So go ahead and treat
         the packet as a success (basically simulate as if it were dropped),
         which will prevent our timers from being messed up. */
      if ((wres != PseudoTcpWriteResult.WR_SUCCESS) && (0 != len))
        return wres;

      priv.t_ack = 0;
      if (len > 0) {
        priv.lastsend = now;
      }
      priv.last_traffic = now;
      priv.bOutgoing = true;

      return PseudoTcpWriteResult.WR_SUCCESS;
    }

    static gboolean
    parse (PseudoTcpSocket self, byte[] _header_buf, gsize header_buf_len,
        byte[] data_buf, gsize data_buf_len)
    {
      Segment seg = new Segment();

      /*union {
        const byte *u8;
        const ushort *u16;
        const uint *u32;
      } header_buf;

      header_buf.u8 = _header_buf;*/

      if (header_buf_len != 24)
        return false;

      using (System.IO.MemoryStream st = new System.IO.MemoryStream(_header_buf))
      using (System.IO.BinaryReader reader = new System.IO.BinaryReader(st))
      {
          seg.conv = reader.ReadUInt32();
          seg.seq = reader.ReadUInt32();
          seg.ack = reader.ReadUInt32();

          reader.ReadByte(); // HERE ONE BYTE IS SKIPPED, BUT C code access to seg.flags = header_buf.u8[13]; => means it is the byte 14, so header_buf[12] (13th byte) is ignored

          seg.flags = (TcpFlags) reader.ReadByte();
          seg.wnd = reader.ReadUInt16();

          seg.tsval = reader.ReadUInt32();
          seg.tsecr = reader.ReadUInt32();

          seg.data = data_buf;
          seg.len = (uint) data_buf_len;
      }

      DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_VERBOSE,
          "Received <CONV={0}><FLG={1}><SEQ={2}:{3}><ACK={4}><WND={5}><TS={6}><TSR={7}><LEN={8}>",
          seg.conv, /*(unsigned)*/seg.flags, seg.seq, seg.seq + seg.len, seg.ack,
          seg.wnd, seg.tsval % 10000, seg.tsecr % 10000, seg.len);

      return process(self, seg);
    }

    /* True iff the @state requires that a FIN has already been sent by this
     * host. */
    static gboolean
    pseudo_tcp_state_has_sent_fin (PseudoTcpState state)
    {
      switch (state) {
      case PseudoTcpState.TCP_LISTEN:
      case PseudoTcpState.TCP_SYN_SENT:
      case PseudoTcpState.TCP_SYN_RECEIVED:
      case PseudoTcpState.TCP_ESTABLISHED:
      case PseudoTcpState.TCP_CLOSE_WAIT:
        return false;
      case PseudoTcpState.TCP_CLOSED:
      case PseudoTcpState.TCP_FIN_WAIT_1:
      case PseudoTcpState.TCP_FIN_WAIT_2:
      case PseudoTcpState.TCP_CLOSING:
      case PseudoTcpState.TCP_TIME_WAIT:
      case PseudoTcpState.TCP_LAST_ACK:
        return true;
      default:
        return false;
      }
    }

    /* True iff the @state requires that a FIN has already been received from the
     * peer. */
    static gboolean
    pseudo_tcp_state_has_received_fin (PseudoTcpState state)
    {
      switch (state) {
      case PseudoTcpState.TCP_LISTEN:
      case PseudoTcpState.TCP_SYN_SENT:
      case PseudoTcpState.TCP_SYN_RECEIVED:
      case PseudoTcpState.TCP_ESTABLISHED:
      case PseudoTcpState.TCP_FIN_WAIT_1:
      case PseudoTcpState.TCP_FIN_WAIT_2:
        return false;
      case PseudoTcpState.TCP_CLOSED:
      case PseudoTcpState.TCP_CLOSING:
      case PseudoTcpState.TCP_TIME_WAIT:
      case PseudoTcpState.TCP_CLOSE_WAIT:
      case PseudoTcpState.TCP_LAST_ACK:
        return true;
      default:
        return false;
      }
    }

    /* True iff the @state requires that a FIN-ACK has already been received from
     * the peer. */
    static gboolean
    pseudo_tcp_state_has_received_fin_ack (PseudoTcpState state)
    {
      switch (state) {
      case PseudoTcpState.TCP_LISTEN:
      case PseudoTcpState.TCP_SYN_SENT:
      case PseudoTcpState.TCP_SYN_RECEIVED:
      case PseudoTcpState.TCP_ESTABLISHED:
      case PseudoTcpState.TCP_FIN_WAIT_1:
      case PseudoTcpState.TCP_FIN_WAIT_2:
      case PseudoTcpState.TCP_CLOSING:
      case PseudoTcpState.TCP_CLOSE_WAIT:
      case PseudoTcpState.TCP_LAST_ACK:
        return false;
      case PseudoTcpState.TCP_CLOSED:
      case PseudoTcpState.TCP_TIME_WAIT:
        return true;
      default:
        return false;
      }
    }

    static gboolean
    process(PseudoTcpSocket self, Segment seg)
    {
      PseudoTcpSocketPrivate priv = self.priv;
      guint32 now;
      SendFlags sflags = SendFlags.sfNone;
      gboolean bIgnoreData;
      gboolean bNewData;
      gboolean bConnect = false;
      gsize snd_buffered;
      gsize available_space;
      guint32 kIdealRefillSize;
      gboolean is_valuable_ack, is_duplicate_ack, is_fin_ack = false;
      gboolean received_fin = false;

      /* If this is the wrong conversation, send a reset!?!
         (with the correct conversation?) */
      if (seg.conv != priv.conv) {
        //if ((seg.flags & FLAG_RST) == 0) {
        //  packet(sock, tcb, seg.ack, 0, FLAG_RST, 0, 0);
        //}
        DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "wrong conversation");
        return false;
      }

      now = get_current_time (self);
      priv.last_traffic = priv.lastrecv = now;
      priv.bOutgoing = false;

      if (priv.state == PseudoTcpState.TCP_CLOSED ||
          (pseudo_tcp_state_has_received_fin_ack (priv.state) && seg.len > 0)) {
        /* Send an RST segment. See: RFC 1122, §4.2.2.13; RFC 793, §3.4, point 3,
         * page 37. We can only send RST if we know the peer knows we’re closed;
         * otherwise this could be a timeout retransmit from them, due to our
         * packets from data through to FIN being dropped. */
        DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL,
            "Segment received while closed; sending RST.");
        if ((seg.flags & TcpFlags.FLAG_RST) == 0) {
          closedown (self, 0, ClosedownSource.CLOSEDOWN_LOCAL);
        }

        return false;
      }

      // Check if this is a reset segment
      if ((seg.flags & TcpFlags.FLAG_RST) != 0) {
        DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "Received RST segment; closing down.");
        closedown (self, ECONNRESET, ClosedownSource.CLOSEDOWN_REMOTE);
        return false;
      }

      // Check for control data
      bConnect = false;
      if ((seg.flags & TcpFlags.FLAG_CTL) != 0) {
        if (seg.len == 0) {
          DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "Missing control code");
          return false;
        } else if (seg.data[0] == CTL_CONNECT) {
          bConnect = true;

#warning Again suboptimal => we should use pointers or streams
          byte[] segdata1 = new byte[seg.len - 1];
          Buffer.BlockCopy(seg.data, 1, segdata1, 0, segdata1.Length);

          parse_options (self, segdata1, seg.len - 1);

          if (priv.state == PseudoTcpState.TCP_LISTEN) {
            set_state (self, PseudoTcpState.TCP_SYN_RECEIVED);
            queue_connect_message (self);
          } else if (priv.state == PseudoTcpState.TCP_SYN_SENT) {
            set_state_established (self);
          }
        } else {
          DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "Unknown control code: %u", seg.data[0]);
          return false;
        }
      }

      // Update timestamp
      if (SMALLER_OR_EQUAL (seg.seq, priv.ts_lastack) &&
          SMALLER (priv.ts_lastack, seg.seq + seg.len)) {
        priv.ts_recent = seg.tsval;
      }

      // Check if this is a valuable ack
      is_valuable_ack = (LARGER(seg.ack, priv.snd_una) &&
          SMALLER_OR_EQUAL(seg.ack, priv.snd_nxt));
      is_duplicate_ack = (seg.ack == priv.snd_una);

      if (is_valuable_ack) {
        guint32 nAcked;
        guint32 nFree;

        // Calculate round-trip time
        if (seg.tsecr != 0) {
#warning rtt was "long" in C code and the cast was not there
          gsize rtt = (gsize) time_diff(now, seg.tsecr);
          if (rtt >= 0) {
            if (priv.rx_srtt == 0) {
              priv.rx_srtt = (uint) rtt;
              priv.rx_rttvar = (uint) rtt / 2;
            } else {
              priv.rx_rttvar = (size_t) ((3 * priv.rx_rttvar +
                  Math.Abs((long)(rtt - priv.rx_srtt))) / 4);
              priv.rx_srtt = (7 * priv.rx_srtt + rtt) / 8;
            }
            priv.rx_rto = bound(MIN_RTO,
                (size_t) (priv.rx_srtt + Math.Max(1LU, 4 * priv.rx_rttvar)), MAX_RTO);

            DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_VERBOSE, "rtt: %ld srtt: %u rttvar: %u rto: %u",
                rtt, priv.rx_srtt, priv.rx_rttvar, priv.rx_rto);
          } else {
            DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "Invalid RTT: %ld", rtt);
            return false;
          }

          priv.last_acked_ts = seg.tsecr;
        }

        priv.snd_wnd = (size_t) (seg.wnd << priv.swnd_scale);

        nAcked = seg.ack - priv.snd_una;
        priv.snd_una = seg.ack;

        priv.rto_base = (priv.snd_una == priv.snd_nxt) ? 0 : now;

        /* ACKs for FIN segments give an increment on nAcked, but there is no
         * corresponding byte to read because the FIN segment is empty (it just has
         * a sequence number). */
        if (nAcked == priv.sbuf.data_length + 1 &&
            pseudo_tcp_state_has_sent_fin (priv.state)) {
          is_fin_ack = true;
          nAcked--;
        }

        pseudo_tcp_fifo_consume_read_data (priv.sbuf, nAcked);

        for (nFree = nAcked; nFree > 0; ) {
          SSegment data;

          g_assert(g_queue_get_length (priv.slist) != 0);
          data = (SSegment ) g_queue_peek_head (priv.slist);

          if (nFree < data.len) {
            data.len -= nFree;
            data.seq += nFree;
            nFree = 0;
          } else {
            if (data.len > priv.largest) {
              priv.largest = data.len;
            }
            nFree -= data.len;
            //g_slice_free (SSegment, data);
            g_queue_pop_head (priv.slist);
          }
        }

        if (priv.dup_acks >= 3) {
          if (LARGER_OR_EQUAL (priv.snd_una, priv.recover)) { // NewReno
            uint nInFlight = priv.snd_nxt - priv.snd_una;
            // (Fast Retransmit)
            priv.cwnd = Math.Min(priv.ssthresh,
                Math.Max (nInFlight, priv.mss) + priv.mss);
            DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "exit recovery cwnd=%d ssthresh=%d nInFlight=%d mss: %d", priv.cwnd, priv.ssthresh, nInFlight, priv.mss);
            priv.fast_recovery = false;
            priv.dup_acks = 0;
          } else {
            int transmit_status;

            DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "recovery retransmit");
            transmit_status = transmit(self, g_queue_peek_head (priv.slist), now);
            if (transmit_status != 0) {
              DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL,
                  "Error transmitting recovery retransmit segment. Closing down.");
              closedown (self, (size_t) transmit_status, ClosedownSource.CLOSEDOWN_LOCAL);
              return false;
            }
            priv.cwnd += (nAcked > priv.mss ? priv.mss : 0) -
                Math.Min(nAcked, priv.cwnd);
          }
        } else {
          priv.dup_acks = 0;
          // Slow start, congestion avoidance
          if (priv.cwnd < priv.ssthresh) {
            priv.cwnd += priv.mss;
          } else {
            priv.cwnd += (size_t) Math.Max(1LU, priv.mss * priv.mss / priv.cwnd);
          }
        }
      } else if (is_duplicate_ack) {
        /* !?! Note, tcp says don't do this... but otherwise how does a
           closed window become open? */
        priv.snd_wnd = (size_t) seg.wnd << priv.swnd_scale;

        // Check duplicate acks
        if (seg.len > 0) {
          // it's a dup ack, but with a data payload, so don't modify priv.dup_acks
        } else if (priv.snd_una != priv.snd_nxt) {
          uint nInFlight;

          priv.dup_acks += 1;
          DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_VERBOSE, "Received dup ack (dups: %u)",
              priv.dup_acks);
          if (priv.dup_acks == 3) { // (Fast Retransmit)
            guint32 transmit_status;

            if (LARGER_OR_EQUAL (priv.snd_una, priv.recover) ||
                seg.tsecr == priv.last_acked_ts) { /* NewReno */
              /* Invoke fast retransmit  RFC3782 section 3 step 1A*/
              DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "enter recovery");
              DEBUG(self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "recovery retransmit");

#warning the cast was not here in C code
              transmit_status = (guint32)transmit(self, g_queue_peek_head (priv.slist),
                  now);
              if (transmit_status != 0) {
                  DEBUG(self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL,
                    "Error transmitting recovery retransmit segment. Closing down.");

                closedown (self, (size_t) transmit_status, ClosedownSource.CLOSEDOWN_LOCAL);
                return false;
              }
              priv.recover = priv.snd_nxt;
              nInFlight = priv.snd_nxt - priv.snd_una;
              priv.ssthresh = Math.Max(nInFlight / 2, 2 * priv.mss);
              DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL,
                  "ssthresh: %u = max((nInFlight: %u / 2), 2 * mss: %u)",
                  priv.ssthresh, nInFlight, priv.mss);
              priv.cwnd = priv.ssthresh + 3 * priv.mss;
              priv.fast_recovery = true;
            } else {
              DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_VERBOSE,
                  "Skipping fast recovery: recover: %u snd_una: %u", priv.recover,
                  priv.snd_una);
            }
          } else if (priv.dup_acks > 3) {
            if (priv.fast_recovery)
              priv.cwnd += priv.mss;
          }
        } else {
          priv.dup_acks = 0;
        }
      }

      // !?! A bit hacky
      if ((priv.state == PseudoTcpState.TCP_SYN_RECEIVED) && !bConnect)
      {
        set_state_established (self);
      }

      /* Check for connection closure. Only pay attention to FIN segments if they
       * are in sequence; otherwise we’ve missed a packet earlier in the stream and
       * need to request retransmission first. */
      if (priv.support_fin_ack) {
        /* @received_fin is set when, and only when, all segments preceding the FIN
         * have been acknowledged. This is to handle the case where the FIN arrives
         * out of order with a preceding data segment. */
        if ((seg.flags & TcpFlags.FLAG_FIN) != 0) {
          priv.rcv_fin = seg.seq;
          DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "Setting rcv_fin = %u", priv.rcv_fin);
        }

        /* For the moment, FIN segments must not contain data. */
        if (((seg.flags & TcpFlags.FLAG_FIN) !=0) && seg.len != 0) {
          DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "FIN segment contained data; ignored");
          return false;
        }

        received_fin = (priv.rcv_nxt != 0 && priv.rcv_nxt + seg.len == priv.rcv_fin);

        /* Update the state machine, implementing all transitions on ‘rcv FIN’ or
         * ‘rcv ACK of FIN’ from RFC 793, Figure 6; and RFC 1122, §4.2.2.8. */
        switch (priv.state) {
        case PseudoTcpState.TCP_ESTABLISHED:
          if (received_fin) {
            /* Received a FIN from the network, RFC 793, §3.5, Case 2.
             * The code below will send an ACK for the FIN. */
             set_state (self, PseudoTcpState.TCP_CLOSE_WAIT);
          }
          break;
        case PseudoTcpState.TCP_CLOSING:
          if (is_fin_ack) {
            /* Handle the ACK of a locally-sent FIN flag. RFC 793, §3.5, Case 3. */
            set_state (self, PseudoTcpState.TCP_TIME_WAIT);
          }
          break;
        case PseudoTcpState.TCP_LAST_ACK:
          if (is_fin_ack) {
            /* Handle the ACK of a locally-sent FIN flag. RFC 793, §3.5, Case 2. */
            set_state_closed (self, 0);
          }
          break;
        case PseudoTcpState.TCP_FIN_WAIT_1:
          if (is_fin_ack && received_fin) {
            /* Simultaneous close with an ACK for a FIN previously sent,
             * RFC 793, §3.5, Case 3. */
            set_state (self, PseudoTcpState.TCP_TIME_WAIT);
          } else if (is_fin_ack) {
            /* Handle the ACK of a locally-sent FIN flag. RFC 793, §3.5, Case 1. */
            set_state (self, PseudoTcpState.TCP_FIN_WAIT_2);
          } else if (received_fin) {
            /* Simultaneous close, RFC 793, §3.5, Case 3. */
            set_state (self, PseudoTcpState.TCP_CLOSING);
          }
          break;
        case PseudoTcpState.TCP_FIN_WAIT_2:
          if (received_fin) {
            /* Local user closed the connection, RFC 793, §3.5, Case 1. */
            set_state (self, PseudoTcpState.TCP_TIME_WAIT);
          }
          break;
        case PseudoTcpState.TCP_LISTEN:
        case PseudoTcpState.TCP_SYN_SENT:
        case PseudoTcpState.TCP_SYN_RECEIVED:
        case PseudoTcpState.TCP_TIME_WAIT:
        case PseudoTcpState.TCP_CLOSED:
        case PseudoTcpState.TCP_CLOSE_WAIT:
          /* Shouldn’t ever hit these cases. */
          if (received_fin) {
            DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL,
               "Unexpected state %u when FIN received", priv.state);
          } else if (is_fin_ack) {
              DEBUG(self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL,
               "Unexpected state %u when FIN-ACK received", priv.state);
          }
          break;
        default:
          DEBUG(self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "Invalid state %u when FIN received",
              priv.state);
          return false;
        }
      } else if ((seg.flags & TcpFlags.FLAG_FIN) != 0) {
          DEBUG(self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL,
            "Invalid FIN received when FIN-ACK support is disabled");
      } else if (is_fin_ack) {
          DEBUG(self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL,
            "Invalid FIN-ACK received when FIN-ACK support is disabled");
      }

      // If we make room in the send queue, notify the user
      // The goal it to make sure we always have at least enough data to fill the
      // window.  We'd like to notify the app when we are halfway to that point.
      kIdealRefillSize = (priv.sbuf_len + priv.rbuf_len) / 2;

      snd_buffered = pseudo_tcp_fifo_get_buffered (priv.sbuf);
      if (priv.bWriteEnable && snd_buffered < kIdealRefillSize) {
        priv.bWriteEnable = false;
        //if (priv.callbacks.PseudoTcpWritable)
          priv.callbacks.PseudoTcpWritable(self, priv.callbacks.user_data);
      }

      /* Conditions where acks must be sent:
       * 1) Segment is too old (they missed an ACK) (immediately)
       * 2) Segment is too new (we missed a segment) (immediately)
       * 3) Segment has data (so we need to ACK!) (delayed)
       * ... so the only time we don't need to ACK, is an empty segment
       * that points to rcv_nxt!
       * 4) Segment has the FIN flag set (immediately) — note that the FIN flag
       *    itself has to be included in the ACK as a numbered byte;
       *    see RFC 793, §3.3. Also see: RFC 793, §3.5.
       */
      if (seg.seq != priv.rcv_nxt) {
        sflags = SendFlags.sfDuplicateAck; // (Fast Recovery)
      } else if (seg.len != 0) {
        if (priv.ack_delay == 0) {
          sflags = SendFlags.sfImmediateAck;
        } else {
          sflags = SendFlags.sfDelayedAck;
        }
      } else if (received_fin) {
        /* FIN flags have a sequence number. Only acknowledge them after all
         * preceding octets have been acknowledged. */
        sflags = SendFlags.sfImmediateAck;
      }

      if (sflags == SendFlags.sfDuplicateAck) {
        if (seg.seq > priv.rcv_nxt) {
            DEBUG(self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "too new");
        } else if (SMALLER_OR_EQUAL(seg.seq + seg.len, priv.rcv_nxt)) {
            DEBUG(self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "too old");
        }
      }

      // Adjust the incoming segment to fit our receive buffer
      if (SMALLER(seg.seq, priv.rcv_nxt)) {
        guint32 nAdjust = priv.rcv_nxt - seg.seq;
        if (nAdjust < seg.len) {
          seg.seq += nAdjust;

          seg.len -= nAdjust;

          byte[] newSegData = new byte[seg.len];

          Buffer.BlockCopy(seg.data, (int) nAdjust, newSegData, 0, newSegData.Length);

          //          seg.data += nAdjust;

          seg.data = newSegData;

        } else {
          seg.len = 0;
        }
      }

      available_space = pseudo_tcp_fifo_get_write_remaining (priv.rbuf);

      if ((seg.seq + seg.len - priv.rcv_nxt) > available_space) {
        guint32 nAdjust = seg.seq + seg.len - priv.rcv_nxt - available_space;
        if (nAdjust < seg.len) {
          seg.len -= nAdjust;
        } else {
          seg.len = 0;
        }
      }

      bIgnoreData = (seg.flags & TcpFlags.FLAG_CTL) != 0;
      if (!priv.support_fin_ack)
        bIgnoreData |= (priv.shutdown != Shutdown.SD_NONE);

      bNewData = false;

      if (seg.len > 0) {
        if (bIgnoreData) {
          if (seg.seq == priv.rcv_nxt) {
            priv.rcv_nxt += seg.len;
          }
        } else {
          guint32 nOffset = seg.seq - priv.rcv_nxt;
          gsize res;

          res = pseudo_tcp_fifo_write_offset (priv.rbuf, seg.data,
              seg.len, nOffset);
          g_assert (res == seg.len);

          if (seg.seq == priv.rcv_nxt) {

            pseudo_tcp_fifo_consume_write_buffer (priv.rbuf, seg.len);
            priv.rcv_nxt += seg.len;
            priv.rcv_wnd -= seg.len;
            bNewData = true;

            RSegment iter = priv.rlist.Count > 0 ? priv.rlist[0] : null;

            while ((priv.rlist.Count > 0) &&
                SMALLER_OR_EQUAL(iter.seq, priv.rcv_nxt)) {

              if (LARGER (iter.seq + iter.len, priv.rcv_nxt)) {
                uint nAdjust = (iter.seq + iter.len) - priv.rcv_nxt;
                sflags = SendFlags.sfImmediateAck; // (Fast Recovery)
                DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "Recovered {0} bytes ({1} . {2})",
                    nAdjust, priv.rcv_nxt, priv.rcv_nxt + nAdjust);
                pseudo_tcp_fifo_consume_write_buffer (priv.rbuf, nAdjust);
                priv.rcv_nxt += nAdjust;
                priv.rcv_wnd -= nAdjust;
              }

              //g_slice_free (RSegment, priv.rlist.data);
              // priv.rlist = g_list_delete_link (priv.rlist, priv.rlist);
              priv.rlist.RemoveAt(0);

              iter = priv.rlist.Count > 0 ? priv.rlist[0] : null;
            }
          } else {
            RSegment rseg = new RSegment();

            DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "Saving {0} bytes ({1} . {2})",
                seg.len, seg.seq, seg.seq + seg.len);
            rseg.seq = seg.seq;
            rseg.len = seg.len;

            RSegment iter = priv.rlist.Count > 0 ? priv.rlist[0] : null;

            int i = 0;

            while (iter != null && SMALLER (iter.seq, rseg.seq)) {
                ++i;
                iter = i < priv.rlist.Count ? priv.rlist[i] : null;
            }

            priv.rlist.Insert(i, rseg);
          }
        }
      }

      if (received_fin) {
        /* FIN flags have a sequence number. */
        priv.rcv_nxt++;
      }


      attempt_send(self, sflags);

      // If we have new data, notify the user
      if (bNewData && priv.bReadEnable) {
        /* priv.bReadEnable = false; — removed so that we’re always notified of
         * incoming pseudo-TCP data, rather than having to read the entire buffer
         * on each readable() callback before the next callback is enabled.
         * (When client-provided buffers are small, this is not possible.) */
        //if (priv.callbacks.PseudoTcpReadable)
          priv.callbacks.PseudoTcpReadable(self, priv.callbacks.user_data);
      }

      return true;
    }

    static int /*gboolean - originally in C, but it does not match in C#*/
    transmit(PseudoTcpSocket self, SSegment segment, guint32 now)
    {
      PseudoTcpSocketPrivate priv = self.priv;
      uint nTransmit = Math.Min(segment.len, priv.mss);

      if (segment.xmit >= ((priv.state == PseudoTcpState.TCP_ESTABLISHED) ? 15 : 30)) {
        DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "too many retransmits");
        return ETIMEDOUT;
      }

      while (true) {
        guint32 seq = segment.seq;
#warning handled as guint8 in C code
        TcpFlags flags = segment.flags;
        PseudoTcpWriteResult wres;

        /* The packet must not have already been acknowledged. */
#warning Uncomment and implement this assert
//        g_assert_cmpuint (segment.seq - priv.snd_una, <=, 1024 * 1024 * 64);

        /* Write out the packet. */
        wres = packet(self, seq, flags,
            segment.seq - priv.snd_una, nTransmit, now);

        if (wres == PseudoTcpWriteResult.WR_SUCCESS)
          break;

        if (wres == PseudoTcpWriteResult.WR_FAIL) {
          DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "packet failed");
          return ECONNABORTED;  /* FIXME: This error code doesn’t quite seem right */
        }

        g_assert(wres == PseudoTcpWriteResult.WR_TOO_LARGE);

        while (true) {
          if (PACKET_MAXIMUMS[priv.msslevel + 1] == 0) {
            DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "MTU too small");
            return EMSGSIZE;
          }
          /* !?! We need to break up all outstanding and pending packets
             and then retransmit!?! */

#warning this cast to gsize was not here in C code
          priv.mss = (gsize) (PACKET_MAXIMUMS[++priv.msslevel] - PACKET_OVERHEAD);
          // I added this... haven't researched actual formula
          priv.cwnd = 2 * priv.mss;

          if (priv.mss < nTransmit) {
            nTransmit = priv.mss;
            break;
          }
        }
        DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "Adjusting mss to %u bytes ", priv.mss);
      }

      if (nTransmit < segment.len) {
        SSegment subseg = new SSegment();
        subseg.seq = segment.seq + nTransmit;
        subseg.len = segment.len - nTransmit;
        subseg.flags = segment.flags;
        subseg.xmit = segment.xmit;

        DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "mss reduced to {0}", priv.mss);

        segment.len = nTransmit;
        g_queue_insert_after (priv.slist,
            g_queue_find (priv.slist, segment), subseg);
        if (subseg.xmit == 0)
          g_queue_insert_after (priv.unsent_slist,
              g_queue_find (priv.unsent_slist, segment), subseg);
      }

      if (segment.xmit == 0) {
        g_assert (g_queue_peek_head (priv.unsent_slist) == segment);
        g_queue_pop_head (priv.unsent_slist);
        priv.snd_nxt += segment.len;

        /* FIN flags require acknowledgement. */
        if (segment.len == 0 && ((segment.flags & TcpFlags.FLAG_FIN) != 0))
          priv.snd_nxt++;
      }
      segment.xmit += 1;

      if (priv.rto_base == 0) {
        priv.rto_base = now;
      }

      return 0;
    }

    static void
    attempt_send(PseudoTcpSocket self, SendFlags sflags)
    {
      PseudoTcpSocketPrivate priv = self.priv;
      uint now = get_current_time (self);
      bool bFirst = true;

      DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "Attempting send with flags %u.", sflags);

      if (time_diff(now, priv.lastsend) > (long) priv.rx_rto) {
        priv.cwnd = priv.mss;
      }


      while (true) {
        guint32 cwnd;
        guint32 nWindow;
        guint32 nInFlight;
        guint32 nUseable;
        guint32 nAvailable;
        gsize snd_buffered;
        SSegment sseg;
        int transmit_status;

        cwnd = priv.cwnd;
        if ((priv.dup_acks == 1) || (priv.dup_acks == 2)) { // Limited Transmit
          cwnd += priv.dup_acks * priv.mss;
        }
        nWindow = Math.Min(priv.snd_wnd, cwnd);
        nInFlight = priv.snd_nxt - priv.snd_una;
        nUseable = (nInFlight < nWindow) ? (nWindow - nInFlight) : 0;
        snd_buffered = pseudo_tcp_fifo_get_buffered (priv.sbuf);
        if (snd_buffered < nInFlight)  /* iff a FIN has been sent */
          nAvailable = 0;
        else
          nAvailable = Math.Min(snd_buffered - nInFlight, priv.mss);

        if (nAvailable > nUseable) {
          if (nUseable * 4 < nWindow) {
            // RFC 813 - avoid SWS
            nAvailable = 0;
          } else {
            nAvailable = nUseable;
          }
        }

        if (bFirst) {
          gsize available_space = pseudo_tcp_fifo_get_write_remaining (priv.sbuf);

          bFirst = false;
          /* COMMENTED OUT
           * DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_VERBOSE,
              "[cwnd: %u  nWindow: %u  nInFlight: %u nAvailable: %u nQueued: %" G_GSIZE_FORMAT " nEmpty: %" G_GSIZE_FORMAT
              "  nWaiting: %zu ssthresh: %u]",
              priv.cwnd, nWindow, nInFlight, nAvailable, snd_buffered,
              available_space, snd_buffered - nInFlight, priv.ssthresh);*/
        }

        if (sflags == SendFlags.sfDuplicateAck) {
          packet(self, priv.snd_nxt, 0, 0, 0, now);
          sflags = SendFlags.sfNone;
          continue;
        }

        if (nAvailable == 0 && sflags != SendFlags.sfFin && sflags != SendFlags.sfRst) {
          if (sflags == SendFlags.sfNone)
            return;

          // If this is an immediate ack, or the second delayed ack
          if ((sflags == SendFlags.sfImmediateAck || sflags == SendFlags.sfDuplicateAck) ||
              priv.t_ack != 0) {
            packet(self, priv.snd_nxt, 0, 0, 0, now);
          } else {
            priv.t_ack = now;
          }
          return;
        }

        // Nagle algorithm
        // If there is data already in-flight, and we haven't a full segment of
        // data ready to send then hold off until we get more to send, or the
        // in-flight data is acknowledged.
        if (priv.use_nagling && sflags != SendFlags.sfFin && sflags != SendFlags.sfRst &&
            (priv.snd_nxt > priv.snd_una) &&
            (nAvailable < priv.mss))  {
          return;
        }

        // Find the next segment to transmit
        sseg = g_queue_peek_head(priv.unsent_slist);

        if (sseg == null)
          return;

        // If the segment is too large, break it into two
        if (sseg.len > nAvailable && sflags != SendFlags.sfFin && sflags != SendFlags.sfRst) {
          SSegment subseg = new SSegment();
          subseg.seq = sseg.seq + nAvailable;
          subseg.len = sseg.len - nAvailable;
          subseg.flags = sseg.flags;

          sseg.len = nAvailable;
          g_queue_insert_after (priv.unsent_slist, 0, subseg);
          g_queue_insert_after (priv.slist, g_queue_find (priv.slist, sseg),
              subseg);
        }

        transmit_status = transmit(self, sseg, now);
        if (transmit_status != 0) {
          DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "transmit failed");

          // TODO: Is this the right thing ?
#warning this cast to gsize was not in C code
          closedown (self, (gsize) transmit_status, ClosedownSource.CLOSEDOWN_REMOTE);
          return;
        }

        if (sflags == SendFlags.sfImmediateAck || sflags == SendFlags.sfDelayedAck)
          sflags = SendFlags.sfNone;
      }
    }

    /* If @source is %CLOSEDOWN_REMOTE, don’t send an RST packet, since closedown()
     * has been called as a result of an RST segment being received.
     * See: RFC 1122, §4.2.2.13. */
    static void
    closedown (PseudoTcpSocket self, guint32 err, ClosedownSource source)
    {
      PseudoTcpSocketPrivate priv = self.priv;

      DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "Closing down socket %p with %s error %u.",
          self, (source == ClosedownSource.CLOSEDOWN_LOCAL) ? "local" : "remote", err);

      if (source == ClosedownSource.CLOSEDOWN_LOCAL && priv.support_fin_ack) {
        queue_rst_message (self);
        attempt_send (self, SendFlags.sfRst);
      } else if (source == ClosedownSource.CLOSEDOWN_LOCAL) {
        priv.shutdown = Shutdown.SD_FORCEFUL;
      }

      /* ‘Cute’ little navigation through the state machine to avoid breaking the
       * invariant that CLOSED can only be reached from TIME-WAIT or LAST-ACK. */
      switch (priv.state) {
      case PseudoTcpState.TCP_LISTEN:
      case PseudoTcpState.TCP_SYN_SENT:
        break;
      case PseudoTcpState.TCP_SYN_RECEIVED:
      case PseudoTcpState.TCP_ESTABLISHED:
        set_state(self, PseudoTcpState.TCP_FIN_WAIT_1);
        goto case PseudoTcpState.TCP_FIN_WAIT_1;
        /* Fall through. */
      case PseudoTcpState.TCP_FIN_WAIT_1:
        set_state (self, PseudoTcpState.TCP_FIN_WAIT_2);
        goto case PseudoTcpState.TCP_FIN_WAIT_2;
        /* Fall through. */
      case PseudoTcpState.TCP_FIN_WAIT_2:
      case PseudoTcpState.TCP_CLOSING:
        set_state (self, PseudoTcpState.TCP_TIME_WAIT);
        break;
      case PseudoTcpState.TCP_CLOSE_WAIT:
        set_state (self, PseudoTcpState.TCP_LAST_ACK);
        break;
      case PseudoTcpState.TCP_LAST_ACK:
      case PseudoTcpState.TCP_TIME_WAIT:
      case PseudoTcpState.TCP_CLOSED:
      default:
        break;
      }

      set_state_closed (self, err);
    }

    static void
    adjustMTU(PseudoTcpSocket self)
    {
      PseudoTcpSocketPrivate priv = self.priv;

      // Determine our current mss level, so that we can adjust appropriately later
      for (priv.msslevel = 0;
           PACKET_MAXIMUMS[priv.msslevel + 1] > 0;
           ++priv.msslevel) {
        if (((guint16)PACKET_MAXIMUMS[priv.msslevel]) <= priv.mtu_advise) {
          break;
        }
      }
      priv.mss = priv.mtu_advise - PACKET_OVERHEAD;
      // !?! Should we reset priv.largest here?
      DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "Adjusting mss to %u bytes", priv.mss);
      // Enforce minimums on ssthresh and cwnd
      priv.ssthresh = Math.Max(priv.ssthresh, 2 * priv.mss);
      priv.cwnd = Math.Max(priv.cwnd, priv.mss);
    }

    static void
    apply_window_scale_option (PseudoTcpSocket self, guint8 scale_factor)
    {
       PseudoTcpSocketPrivate priv = self.priv;

       priv.swnd_scale = scale_factor;
       DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "Setting scale factor to %u", scale_factor);
    }

    static void
    apply_fin_ack_option (PseudoTcpSocket self)
    {
      PseudoTcpSocketPrivate priv = self.priv;

      priv.support_fin_ack = true;
    }

    static void
    apply_option (PseudoTcpSocket self, TcpOption kind, guint8[] data,
        guint32 len)
    {
      switch (kind) {
      case TcpOption.TCP_OPT_MSS:
        DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL,
            "Peer specified MSS option which is not supported.");
        // TODO: Implement.
        break;
      case TcpOption.TCP_OPT_WND_SCALE:
        // Window scale factor.
        // http://www.ietf.org/rfc/rfc1323.txt
        if (len != 1) {
          DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "Invalid window scale option received.");
          return;
        }
        apply_window_scale_option(self, data[0]);
        break;
      case TcpOption.TCP_OPT_FIN_ACK:
        // FIN-ACK support.
        DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "FIN-ACK support enabled.");
        apply_fin_ack_option (self);
        break;
      case TcpOption.TCP_OPT_EOL:
      case TcpOption.TCP_OPT_NOOP:
        /* Nothing to do. */
        break;
      default:
        DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "Invalid TCP option %u", kind);
        break;
      }
    }


    static void
    parse_options (PseudoTcpSocket self, byte[] data, guint32 len)
    {
      PseudoTcpSocketPrivate priv = self.priv;
      gboolean has_window_scaling_option = false;
      gboolean has_fin_ack_option = false;
      guint32 pos = 0;

      // See http://www.freesoft.org/CIE/Course/Section4/8.htm for
      // parsing the options list.
      while (pos < len) {
#warning it was guint8 originally
        TcpOption kind = TcpOption.TCP_OPT_EOL;
        guint8 opt_len;

        if (len < pos + 1)
          return;

#warning this cast was not needed in C
        kind = (TcpOption) data[pos];
        pos++;

        if (kind == TcpOption.TCP_OPT_EOL) {
          // End of option list.
          break;
        } else if (kind == TcpOption.TCP_OPT_NOOP) {
          // No op.
          continue;
        }

        if (len < pos + 1)
          return;

        // Length of this option.
        opt_len = data[pos];
        pos++;

        if (len < pos + opt_len)
          return;

        // Content of this option.
        if (opt_len <= len - pos) {
#warning do not copy bytes
          byte[] datapos = new byte[opt_len];
          Buffer.BlockCopy(data, (int) pos, datapos, 0, datapos.Length);

          apply_option (self, kind, datapos, opt_len);
          pos += opt_len;
        } else {
          DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "Invalid option length received.");
          return;
        }

        if (kind == TcpOption.TCP_OPT_WND_SCALE)
          has_window_scaling_option = true;
        else if (kind == TcpOption.TCP_OPT_FIN_ACK)
          has_fin_ack_option = true;
      }

      if (!has_window_scaling_option) {
        DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "Peer doesn't support window scaling");
        if (priv.rwnd_scale > 0) {
          // Peer doesn't support TCP options and window scaling.
          // Revert receive buffer size to default value.
          resize_receive_buffer (self, DEFAULT_RCV_BUF_SIZE);
          priv.swnd_scale = 0;
        }
      }

      if (!has_fin_ack_option) {
        DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "Peer doesn't support FIN-ACK");
        priv.support_fin_ack = false;
      }
    }

    static void
    resize_send_buffer (PseudoTcpSocket self, guint32 new_size)
    {
      PseudoTcpSocketPrivate priv = self.priv;

      priv.sbuf_len = new_size;
      pseudo_tcp_fifo_set_capacity (priv.sbuf, new_size);
    }


    static void
    resize_receive_buffer (PseudoTcpSocket self, guint32 new_size)
    {
      PseudoTcpSocketPrivate priv = self.priv;
      guint8 scale_factor = 0;
      gboolean result;
      gsize available_space;

      if (priv.rbuf_len == new_size)
        return;

      // Determine the scale factor such that the scaled window size can fit
      // in a 16-bit unsigned integer.
      while (new_size > 0xFFFF) {
        ++scale_factor;
        new_size >>= 1;
      }

      // Determine the proper size of the buffer.
      new_size <<= scale_factor;
      result = pseudo_tcp_fifo_set_capacity (priv.rbuf, new_size);

      // Make sure the new buffer is large enough to contain data in the old
      // buffer. This should always be true because this method is called either
      // before connection is established or when peers are exchanging connect
      // messages.
      g_assert(result);
      priv.rbuf_len = new_size;
      priv.rwnd_scale = scale_factor;
      priv.ssthresh = new_size;

      available_space = pseudo_tcp_fifo_get_write_remaining (priv.rbuf);
      priv.rcv_wnd = available_space;
    }

    gint
    pseudo_tcp_socket_get_available_bytes (PseudoTcpSocket self)
    {
      PseudoTcpSocketPrivate priv = self.priv;

#warning this cast to gint was not in C code
      return (gint) pseudo_tcp_fifo_get_buffered (priv.rbuf);
    }

    bool
    pseudo_tcp_socket_can_send (PseudoTcpSocket self)
    {
      return (pseudo_tcp_socket_get_available_send_space (self) > 0);
    }

    gsize
    pseudo_tcp_socket_get_available_send_space (PseudoTcpSocket self)
    {
      PseudoTcpSocketPrivate priv = self.priv;
      gsize ret;

      if (!pseudo_tcp_state_has_sent_fin (priv.state)) {
        ret = pseudo_tcp_fifo_get_write_remaining (priv.sbuf);
      } else {
        ret = 0;
      }

      if (ret == 0)
        priv.bWriteEnable = true;

      return ret;
    }

    /* State names are capitalised and formatted as in RFC 793. */
    static string
    pseudo_tcp_state_get_name (PseudoTcpState state)
    {
      switch (state) {
      case PseudoTcpState.TCP_LISTEN: return "LISTEN";
      case PseudoTcpState.TCP_SYN_SENT: return "SYN-SENT";
      case PseudoTcpState.TCP_SYN_RECEIVED: return "SYN-RECEIVED";
      case PseudoTcpState.TCP_ESTABLISHED: return "ESTABLISHED";
      case PseudoTcpState.TCP_CLOSED: return "CLOSED";
      case PseudoTcpState.TCP_FIN_WAIT_1: return "FIN-WAIT-1";
      case PseudoTcpState.TCP_FIN_WAIT_2: return "FIN-WAIT-2";
      case PseudoTcpState.TCP_CLOSING: return "CLOSING";
      case PseudoTcpState.TCP_TIME_WAIT: return "TIME-WAIT";
      case PseudoTcpState.TCP_CLOSE_WAIT: return "CLOSE-WAIT";
      case PseudoTcpState.TCP_LAST_ACK: return "LAST-ACK";
      default: return "UNKNOWN";
      }
    }

    static void
    set_state (PseudoTcpSocket self, PseudoTcpState new_state)
    {
      PseudoTcpSocketPrivate priv = self.priv;
      PseudoTcpState old_state = priv.state;

      if (new_state == old_state)
        return;

      DEBUG (self, PseudoTcpDebugLevel.PSEUDO_TCP_DEBUG_NORMAL, "State %s → %s.",
          pseudo_tcp_state_get_name (old_state),
          pseudo_tcp_state_get_name (new_state));

#warning need to uncomment this code
      /* Check whether it’s a valid state transition. */
    /*#define TRANSITION(OLD, NEW) \
        (old_state == TCP_##OLD && \
         new_state == TCP_##NEW)

      // Valid transitions. See: RFC 793, p23; RFC 1122, §4.2.2.8. 
      g_assert (// RFC 793, p23.
                TRANSITION (CLOSED, SYN_SENT) ||
                TRANSITION (SYN_SENT, CLOSED) ||
                TRANSITION (CLOSED, LISTEN) ||
                TRANSITION (LISTEN, CLOSED) ||
                TRANSITION (LISTEN, SYN_SENT) ||
                TRANSITION (LISTEN, SYN_RECEIVED) ||
                TRANSITION (SYN_SENT, SYN_RECEIVED) ||
                TRANSITION (SYN_RECEIVED, ESTABLISHED) ||
                TRANSITION (SYN_SENT, ESTABLISHED) ||
                TRANSITION (SYN_RECEIVED, FIN_WAIT_1) ||
                TRANSITION (ESTABLISHED, FIN_WAIT_1) ||
                TRANSITION (ESTABLISHED, CLOSE_WAIT) ||
                TRANSITION (FIN_WAIT_1, FIN_WAIT_2) ||
                TRANSITION (FIN_WAIT_1, CLOSING) ||
                TRANSITION (CLOSE_WAIT, LAST_ACK) ||
                TRANSITION (FIN_WAIT_2, TIME_WAIT) ||
                TRANSITION (CLOSING, TIME_WAIT) ||
                TRANSITION (LAST_ACK, CLOSED) ||
                TRANSITION (TIME_WAIT, CLOSED) ||
                // RFC 1122, §4.2.2.8.
                TRANSITION (SYN_RECEIVED, LISTEN) ||
                TRANSITION (FIN_WAIT_1, TIME_WAIT));

    //#undef TRANSITION*/

      priv.state = new_state;
    }

    static void
    set_state_established (PseudoTcpSocket self)
    {
      PseudoTcpSocketPrivate priv = self.priv;

      set_state (self, PseudoTcpState.TCP_ESTABLISHED);

      adjustMTU (self);
      //if (priv.callbacks.PseudoTcpOpened)
        priv.callbacks.PseudoTcpOpened(self, priv.callbacks.user_data);
    }

    /* (err == 0) means no error. */
    static void
    set_state_closed (PseudoTcpSocket self, guint32 err)
    {
      PseudoTcpSocketPrivate priv = self.priv;

      set_state (self, PseudoTcpState.TCP_CLOSED);

      /* Only call the callback if there was an error. */
      if (err != 0)
        priv.callbacks.PseudoTcpClosed (self, err, priv.callbacks.user_data);
    }

    bool
    pseudo_tcp_socket_is_closed (PseudoTcpSocket self)
    {
      PseudoTcpSocketPrivate priv = self.priv;

      return (priv.state == PseudoTcpState.TCP_CLOSED);
    }

    bool
    pseudo_tcp_socket_is_closed_remotely (PseudoTcpSocket self)
    {
      PseudoTcpSocketPrivate priv = self.priv;

      return pseudo_tcp_state_has_received_fin (priv.state);
    }
}