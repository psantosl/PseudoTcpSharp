using System;
using System.Threading;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using System.Text;

using PseudoTcp;

using NUnit.Framework;

namespace Tests
{
    [TestFixture]
    public class PseudoTcpTests
    {
        [Test]
        public void BasicTest()
        {
            PseudoTcpSocket.PseudoTcpCallbacks cbsLeft = new PseudoTcpSocket.PseudoTcpCallbacks();
            PseudoTcpSocket.PseudoTcpCallbacks cbsRight = new PseudoTcpSocket.PseudoTcpCallbacks();

            PseudoTcpSocket leftSocket = PseudoTcpSocket.pseudo_tcp_socket_new(0, cbsLeft);
            PseudoTcpSocket rightSocket = PseudoTcpSocket.pseudo_tcp_socket_new(0, cbsRight);

            Common common = new Common(leftSocket, rightSocket);

            byte[] source;

            string initial = "This is a new text that has to be read and now some bytes";

            using (MemoryStream st = new MemoryStream())
            using (BinaryWriter writer =  new BinaryWriter(st))
            {
                writer.Write(initial);
                writer.Write(new byte[1024 * 5]);
                writer.Write(initial);

                source = st.ToArray();
            }

            Left left = new Left(source, common);
            cbsLeft.PseudoTcpOpened = left.LeftOpened;
            cbsLeft.PseudoTcpReadable = null;
            cbsLeft.PseudoTcpWritable = left.Writable;
            cbsLeft.PseudoTcpClosed = common.Closed;
            cbsLeft.WritePacket = common.WritePacket;

            Right right = new Right(left);
            cbsRight.PseudoTcpOpened = null;
            cbsRight.PseudoTcpReadable = right.Readable;
            cbsRight.PseudoTcpWritable = null;
            cbsRight.PseudoTcpClosed = common.Closed;
            cbsRight.WritePacket = common.WritePacket;

            leftSocket.pseudo_tcp_socket_notify_mtu(1496);
            rightSocket.pseudo_tcp_socket_notify_mtu(1496);

            leftSocket.pseudo_tcp_socket_connect();

            common.AdjustClock(leftSocket);
            common.AdjustClock(rightSocket);

            while (!left.Eof() || right.TotalWrote < source.Length)
            {
                Thread.Sleep(500);
            }

            byte[] dst = right.GetData();

            using (MemoryStream st = new MemoryStream(dst))
            using (BinaryReader reader = new BinaryReader(st))
            {
                string s = reader.ReadString();

                byte[] b = reader.ReadBytes(1024 * 5);

                string s2 = reader.ReadString();

                Assert.AreEqual(initial, s, "initial string not ok");
                Assert.AreEqual(initial, s2, "final string not ok");
            }
        }

        class Common
        {
            internal Common(PseudoTcpSocket left, PseudoTcpSocket right)
            {
                mLeft = left;
                mRight = right;
            }

            internal PseudoTcpSocket.PseudoTcpWriteResult WritePacket(
                PseudoTcp.PseudoTcpSocket sock,
                byte[] buffer,
                uint len,
                object user_data)
            {
                int drop_rate = mRnd.Next(100);

                if (drop_rate < 15)
                {
                    Console.WriteLine ("*********************Dropping packet from {0}", GetName(sock));
                    return PseudoTcpSocket.PseudoTcpWriteResult.WR_SUCCESS;
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
                        if (sock == mLeft)
                            Console.WriteLine("Left->Right {0}", newBuffer.Length);
                        else
                            Console.WriteLine("Right->Left {0}", newBuffer.Length);

                        other.pseudo_tcp_socket_notify_packet(newBuffer, (uint)newBuffer.Length);
                        AdjustClock(other);

                        timer.Dispose();
                    },
                    null,
                    (long)0,
                    Timeout.Infinite);

                return PseudoTcpSocket.PseudoTcpWriteResult.WR_SUCCESS;
            }

            internal void AdjustClock(PseudoTcp.PseudoTcpSocket sock)
            {
                ulong timeout = 0;

                if (sock.pseudo_tcp_socket_get_next_clock(ref timeout))
                {
                    uint now = PseudoTcpSocket.g_get_monotonic_time();

                    if (now < timeout)
                        timeout -= now;
                    else
                        timeout = now - timeout;

                    if (timeout > 900)
                        timeout = 100;

                    Console.WriteLine ("Socket {0}: Adjusting clock to {1} ms", GetName(sock), timeout);

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

            internal void Closed(PseudoTcpSocket sock, uint err, object data)
            {
                Console.WriteLine("Closed {0} - err: {1}", GetName(sock), err);
            }

            void NotifyClock(PseudoTcp.PseudoTcpSocket sock)
            {
                //g_debug ("Socket %p: Notifying clock", sock);
                sock.pseudo_tcp_socket_notify_clock();
                AdjustClock(sock);
            }

            string GetName(PseudoTcp.PseudoTcpSocket sock)
            {
                if (sock == mLeft)
                    return "Left";

                return "Right";
            }

            PseudoTcp.PseudoTcpSocket mLeft;
            PseudoTcp.PseudoTcpSocket mRight;

            Random mRnd = new Random();
        }

        class Right
        {
            internal Right(Left left)
            {
                mLeft = left;
            }

            internal void Readable(PseudoTcp.PseudoTcpSocket sock, object data)
            {
                byte[] buf = new byte[1024];
                int len;

                do
                {
                    len = sock.pseudo_tcp_socket_recv(buf, (uint)buf.Length);

                    if (len < 0)
                        break;

                    if (len == 0)
                    {
                        sock.pseudo_tcp_socket_close(false);

                        break;
                    }

                    Console.WriteLine("Right: Read {0} bytes", len);
                    mStream.Write(buf, 0, len);

                    mTotalWroteToRight += len;

                    Assert.IsTrue(mTotalWroteToRight <= mLeft.TotalReadFromLeft);
                    // g_debug ("Written %d bytes, need %d bytes", total_wrote, total_read);

                    if (mTotalWroteToRight == mLeft.TotalReadFromLeft && mLeft.Eof())
                    {
                        //g_assert (reading_done);
                        sock.pseudo_tcp_socket_close(false);
                    }

                } while (len > 0);

                if (len == -1 &&
                    PseudoTcpSocket.pseudo_tcp_socket_get_error(sock) != PseudoTcpSocket.EWOULDBLOCK)
                {
                    Assert.Fail("Error reading from right socket {0}", PseudoTcpSocket.pseudo_tcp_socket_get_error(sock));
                }
            }

            internal int TotalWrote
            {
                get { return mTotalWroteToRight; }
            }

            internal byte[] GetData()
            {
                return mStream.ToArray();
            }

            int mTotalWroteToRight;

            MemoryStream mStream = new MemoryStream();
            Left mLeft;
        }

        class Left
        {
            internal Left(byte[] data, Common common)
            {
                mStream = new MemoryStream(data);
                mCommon = common;
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
                        sock.pseudo_tcp_socket_close(false);
                        break;
                    }

                    wlen = sock.pseudo_tcp_socket_send(buf, (uint)len);
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

                mCommon.AdjustClock(sock);
            }

            int mTotalReadFromLeft;

            MemoryStream mStream;

            Common mCommon;
        }
    }
}