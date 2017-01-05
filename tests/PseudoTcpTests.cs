using System;
using System.Threading;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using System.Text;

using NUnit.Framework;

namespace Tests
{
    [TestFixture]
    public class PseudoTcpTests
    {
        [Test]
        public void BasicTest()
        {
            PseudoTcp.PseudoTcpCallbacks cbsLeft = new PseudoTcp.PseudoTcpCallbacks();
            PseudoTcp.PseudoTcpCallbacks cbsRight = new PseudoTcp.PseudoTcpCallbacks();

            PseudoTcp.PseudoTcpSocket leftSocket = PseudoTcp.pseudo_tcp_socket_new(0, cbsLeft);
            PseudoTcp.PseudoTcpSocket rightSocket = PseudoTcp.pseudo_tcp_socket_new(0, cbsRight);

            Common common = new Common(leftSocket, rightSocket);

            Left left = new Left(new byte[1024]);

            cbsLeft.PseudoTcpOpened = left.LeftOpened;
            cbsLeft.PseudoTcpReadable = null;
            cbsLeft.PseudoTcpWritable = left.Writable;
            cbsLeft.PseudoTcpClosed = Common.Closed;
            cbsLeft.WritePacket = common.WritePacket;

            Right right = new Right();
            cbsRight.PseudoTcpOpened = null;
            cbsRight.PseudoTcpReadable = right.Readable;
            cbsRight.PseudoTcpWritable = null;
            cbsRight.PseudoTcpClosed = Common.Closed;
            cbsRight.WritePacket = common.WritePacket;

            PseudoTcp.pseudo_tcp_socket_init(leftSocket);
            PseudoTcp.pseudo_tcp_socket_init(rightSocket);

            PseudoTcp.pseudo_tcp_socket_notify_mtu(leftSocket, 1496);
            PseudoTcp.pseudo_tcp_socket_notify_mtu(rightSocket, 1496);

            PseudoTcp.pseudo_tcp_socket_connect(leftSocket);

            Common.AdjustClock(leftSocket);
            Common.AdjustClock(rightSocket);

            while (!left.Eof() && right.TotalWrote < 1024)
            {
                Thread.Sleep(100);
            }
        }

        class Common
        {
            internal Common(PseudoTcp.PseudoTcpSocket left, PseudoTcp.PseudoTcpSocket right)
            {
                mLeft = left;
                mRight = right;
            }

            internal PseudoTcp.PseudoTcpWriteResult WritePacket(
                PseudoTcp.PseudoTcpSocket sock,
                byte[] buffer,
                uint len,
                object user_data)
            {
                Random rnd = new Random();

                int drop_rate = rnd.Next() % 100;

                if (drop_rate < 5)
                {
                    // g_debug ("*********************Dropping packet (%d) from %p", drop_rate, sock);
                    return PseudoTcp.PseudoTcpWriteResult.WR_SUCCESS;
                }

                byte[] newBuffer = new byte[len];
                Buffer.BlockCopy(buffer, 0, newBuffer, 0, (int)len);

                PseudoTcp.PseudoTcpSocket other = mLeft;

                if (sock == mLeft)
                    other = mRight;

                Timer timer = null;
                timer = new System.Threading.Timer(
                    (obj) =>
                    {
                        PseudoTcp.pseudo_tcp_socket_notify_packet(other, newBuffer, (uint)newBuffer.Length);
                        AdjustClock(other);

                        timer.Dispose();
                    },
                    null,
                    (long)0,
                    Timeout.Infinite);

                return PseudoTcp.PseudoTcpWriteResult.WR_SUCCESS;
            }

            static internal void AdjustClock(PseudoTcp.PseudoTcpSocket sock)
            {
                ulong timeout = 0;

                if (PseudoTcp.pseudo_tcp_socket_get_next_clock(sock, ref timeout))
                {
                    timeout = PseudoTcp.g_get_monotonic_time();
                    // g_debug ("Socket %p: Adjusting clock to %" G_GUINT64_FORMAT " ms", sock, timeout);

                    Timer timer = null;
                    timer = new System.Threading.Timer(
                        (obj) =>
                        {
                            NotifyClock(sock);
                            timer.Dispose();
                        },
                        null,
                        (long)timeout,
                        Timeout.Infinite);
                }
                else
                {
                    /*left_closed = true;

                    if (left_closed && right_closed)
                        g_main_loop_quit (mainloop);*/
                }
            }

            static internal void Closed(PseudoTcp.PseudoTcpSocket sock, uint err, object data)
            {
                //
            }

            static void NotifyClock(PseudoTcp.PseudoTcpSocket sock)
            {
                //g_debug ("Socket %p: Notifying clock", sock);
                PseudoTcp.pseudo_tcp_socket_notify_clock(sock);
                AdjustClock(sock);
            }

            PseudoTcp.PseudoTcpSocket mLeft;
            PseudoTcp.PseudoTcpSocket mRight;
        }

        class Right
        {
            internal void Readable(PseudoTcp.PseudoTcpSocket sock, object data)
            {
                byte[] buf = new byte[1024];
                int len;

                do
                {
                    len = PseudoTcp.pseudo_tcp_socket_recv (sock, buf, (uint) buf.Length);

                    if (len == 0)
                    {
                        PseudoTcp.pseudo_tcp_socket_close (sock, false);

                        break;
                    }

                    // g_debug ("Read %d bytes", len);
                    mStream.Write(buf, 0, len);

                    mTotalWroteToRight += len;

                    Assert.IsTrue(mTotalWroteToRight <= mLeft.TotalReadFromLeft);
                    // g_debug ("Written %d bytes, need %d bytes", total_wrote, total_read);

                    if (mTotalWroteToRight == mLeft.TotalReadFromLeft && mLeft.Eof())
                    {
                        //g_assert (reading_done);
                        PseudoTcp.pseudo_tcp_socket_close (sock, false);
                    }

                } while (len > 0);

                if (len == -1 &&
                    PseudoTcp.pseudo_tcp_socket_get_error(sock) != PseudoTcp.EWOULDBLOCK)
                {
                    Assert.Fail("Error reading from right socket {0}", PseudoTcp.pseudo_tcp_socket_get_error(sock));
                }
            }

            internal int TotalWrote
            {
                get { return mTotalWroteToRight; }
            }

            int mTotalWroteToRight;

            MemoryStream mStream = new MemoryStream();
            Left mLeft;
        }

        class Left
        {
            internal Left(byte[] data)
            {
                mStream = new MemoryStream(data);
            }

            internal void LeftOpened(PseudoTcp.PseudoTcpSocket sock, object data)
            {
                WriteToSock(sock);
            }

            internal void Writable (PseudoTcp.PseudoTcpSocket sock, object data)
            {
                //g_debug ("Socket %p Writable", sock);
                WriteToSock(sock);
            }

            internal int TotalReadFromLeft
            {
                get { return mTotalReadFromLeft; }
            }

            internal bool Eof()
            {
                return mTotalReadFromLeft == mStream.Length;
            }

            void WriteToSock(PseudoTcp.PseudoTcpSocket sock)
            {
                byte[] buf = new byte[1024];
                int len;
                int wlen;
                int total = 0;

                while (true)
                {
                    len = mStream.Read(buf, 0, buf.Length);
                    if (len == 0)
                    {
                        // reading_done = TRUE;
                        PseudoTcp.pseudo_tcp_socket_close (sock, false);
                        break;
                    }

                    wlen = PseudoTcp.pseudo_tcp_socket_send (sock, buf, (uint) len);
                    total += wlen;
                    mTotalReadFromLeft += wlen;

                    if (wlen < len)
                    {
                        // fseek (in, wlen - len, SEEK_CUR);

                        mStream.Position += wlen - len; // go back to later reread what couldn't be sent

                        // g_assert (!feof (in));
                        // g_debug ("Socket queue full after %d bytes written", total);
                        break;
                    }
                }

                Common.AdjustClock(sock);
            }

            int mTotalReadFromLeft;

            MemoryStream mStream;
        }
    }
}