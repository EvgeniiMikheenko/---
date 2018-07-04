using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Threading;
using System.Runtime.InteropServices;
using Modbus.Device;
using System.Xml;

namespace WindowsFormsApplication1
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();


            is_connected = false;
            stateForm = WorkState.Idle;


            Form_Update = new BackgroundWorker();
            Form_Update.WorkerSupportsCancellation = true;
            Form_Update.WorkerReportsProgress = true;
            Form_Update.DoWork += Form_Update_DoWork;
            Form_Update.ProgressChanged += Form_Update_ProgressChanged;

            Form_Update.RunWorkerAsync();



            m_mbMaster = ModbusSerialMaster.CreateRtu(COM);
            m_mbMaster.Transport.ReadTimeout = 1000;
            m_mbMaster.Transport.Retries = 5;
            m_mbMaster.Transport.SlaveBusyUsesRetryCount = true;
            m_mbMaster.Transport.WaitToRetryMilliseconds = 100;

            m_bgwModbusUpdater = new BackgroundWorker();
            m_bgwModbusUpdater.WorkerSupportsCancellation = true;
            m_bgwModbusUpdater.WorkerReportsProgress = true;
            m_bgwModbusUpdater.DoWork += M_bgwModbusUpdater_DoWork;
            m_bgwModbusUpdater.ProgressChanged += M_bgwModbusUpdater_ProgressChanged;
            m_bgwModbusUpdater.RunWorkerAsync();



            Alarm_lbl.Left = this.Width / 2 - Alarm_lbl.Width / 2;
            Rdy_lbl.Left = this.Width / 2 - Rdy_lbl.Width / 2;

            CHB_btn.Checked = false;

            Parce_Settings();

            gfgf = true;

        }


        ushort Read_reg;
        ushort[] ID = new ushort[6];
        ushort[] date_regs = new ushort[3];
        ushort[] src_num_regs = new ushort[2];
        ushort ver_reg;
        ushort[] md5_regs = new ushort[8];  

        string Pos_Trsh_clr;
        string Pos_Trsh_msg;
        string Pos_Trsh_txt;

        string Pos_Dng_clr;
        string Pos_Dng_msg;
        string Pos_Dng_txt;

        string Neg_Trsh_clr;
        string Neg_Trsh_msg;
        string Neg_Trsh_txt;

        string Neg_Dng_clr;
        string Neg_Dng_msg;
        string Neg_Dng_txt;

        string No_Alrm_clr;
        string No_Alrm_msg;
        string No_Alrm_txt;

        public void Parce_Settings()
        {




            try
            {
                XmlDocument xDoc = new XmlDocument();
                xDoc.Load("Settings.xml");

                XmlElement xRoot = xDoc.DocumentElement;
                foreach (XmlElement xnode in xRoot)
                {
                    XmlNode attr = xnode.Attributes.GetNamedItem("name");
                    if (attr != null)
                    {
                        if (attr.Value == "Pos_Tresh")
                        {
                            foreach (XmlNode childnode in xnode.ChildNodes)
                            {
                                // если узел - company
                                if (childnode.Name == "back_color")
                                {
                                    Pos_Trsh_clr = childnode.InnerText;
                                }
                                // если узел age
                                if (childnode.Name == "message")
                                {
                                    Pos_Trsh_msg = childnode.InnerText;
                                }
                                if (childnode.Name == "text_color")
                                {
                                    Pos_Trsh_txt = childnode.InnerText;
                                }
                            }
                        }
                        if (attr.Value == "Pos_Dng")
                        {
                            foreach (XmlNode childnode in xnode.ChildNodes)
                            {
                                // если узел - company
                                if (childnode.Name == "back_color")
                                {
                                    Pos_Dng_clr = childnode.InnerText;
                                }
                                // если узел age
                                if (childnode.Name == "message")
                                {
                                    Pos_Dng_msg = childnode.InnerText;
                                }
                                if (childnode.Name == "text_color")
                                {
                                    Pos_Dng_txt = childnode.InnerText;
                                }
                            }
                        }


                        if (attr.Value == "Neg_Tresh")
                        {
                            foreach (XmlNode childnode in xnode.ChildNodes)
                            {
                                // если узел - company
                                if (childnode.Name == "back_color")
                                {
                                    Neg_Trsh_clr = childnode.InnerText;
                                }
                                // если узел age
                                if (childnode.Name == "message")
                                {
                                    Neg_Trsh_msg = childnode.InnerText;
                                }
                                if (childnode.Name == "text_color")
                                {
                                    Neg_Trsh_txt = childnode.InnerText;
                                }
                            }
                        }
                        if (attr.Value == "Neg_Dng")
                        {
                            foreach (XmlNode childnode in xnode.ChildNodes)
                            {
                                // если узел - company
                                if (childnode.Name == "back_color")
                                {
                                    Neg_Dng_clr = childnode.InnerText;
                                }
                                // если узел age
                                if (childnode.Name == "message")
                                {
                                    Neg_Dng_msg = childnode.InnerText;
                                }
                                if (childnode.Name == "text_color")
                                {
                                    Neg_Dng_txt = childnode.InnerText;
                                }
                            }
                        }

                        if (attr.Value == "No_Alrm")
                        {
                            foreach (XmlNode childnode in xnode.ChildNodes)
                            {
                                // если узел - company
                                if (childnode.Name == "back_color")
                                {
                                    No_Alrm_clr = childnode.InnerText;
                                }
                                // если узел age
                                if (childnode.Name == "message")
                                {
                                    No_Alrm_msg = childnode.InnerText;
                                }
                                if (childnode.Name == "text_color")
                                {
                                    No_Alrm_txt = childnode.InnerText;
                                }
                            }
                        }

                    }


                }
            }
            catch (Exception ex)
            { };
        }
        bool gfgf;
         ushort stt = 1;

        public void Chec(ushort state)
        {
            switch(stt)
            {
                case 0:
                    Alarm_panel.Invoke((MethodInvoker)delegate
                    {
                        Alarm_panel.BackColor = Color.FromName(No_Alrm_clr);
                    });
                    Alarm_lbl.Invoke((MethodInvoker)delegate
                    {
                        Alarm_lbl.ForeColor = Color.FromName(No_Alrm_txt);
                        Alarm_lbl.Text = No_Alrm_msg;
                        Alarm_lbl.Left = this.Width / 2 - Alarm_lbl.Width / 2;
                    });
                    break;
                case 1:
                    Alarm_panel.Invoke((MethodInvoker)delegate
                    {
                        Alarm_panel.BackColor = Color.FromName(Pos_Trsh_clr);
                    });
                    Alarm_lbl.Invoke((MethodInvoker)delegate
                    {
                        Alarm_lbl.ForeColor = Color.FromName(Pos_Trsh_txt);
                        Alarm_lbl.Text = Pos_Trsh_msg;
                        Alarm_lbl.Left = this.Width / 2 - Alarm_lbl.Width / 2;
                    });
                    break;
                case 2:
                    Alarm_panel.Invoke((MethodInvoker)delegate
                    {
                        Alarm_panel.BackColor = Color.FromName(Pos_Dng_clr);
                    });
                    Alarm_lbl.Invoke((MethodInvoker)delegate
                    {
                        Alarm_lbl.ForeColor = Color.FromName(Pos_Dng_txt);
                        Alarm_lbl.Text = Pos_Dng_msg;
                        Alarm_lbl.Left = this.Width / 2 - Alarm_lbl.Width / 2;
                    });
                    break;
                case 3:
                    Alarm_panel.Invoke((MethodInvoker)delegate
                    {
                        Alarm_panel.BackColor = Color.FromName(Neg_Trsh_clr);
                    });
                    Alarm_lbl.Invoke((MethodInvoker)delegate
                    {
                        Alarm_lbl.ForeColor = Color.FromName(Neg_Trsh_txt);
                        Alarm_lbl.Text = Neg_Trsh_msg;
                        Alarm_lbl.Left = this.Width / 2 - Alarm_lbl.Width / 2;
                    });
                    break;
                case 4:
                    Alarm_panel.Invoke((MethodInvoker)delegate
                    {
                        Alarm_panel.BackColor = Color.FromName(Neg_Dng_clr);
                    });
                    Alarm_lbl.Invoke((MethodInvoker)delegate
                    {
                        Alarm_lbl.ForeColor = Color.FromName(Neg_Dng_txt);
                        Alarm_lbl.Text = Neg_Dng_msg;
                        Alarm_lbl.Left = this.Width / 2 - Alarm_lbl.Width / 2;
                    });
                    break;
            }


            Rdy_lbl.Invoke((MethodInvoker)delegate
            {
                Rdy_lbl.Left = this.Width / 2 - Rdy_lbl.Width / 2;
            });


        }

        private void Form_Update_DoWork(object sender, DoWorkEventArgs e)
        {
            while (true)
            {

                //Alarm_lbl.Invoke((MethodInvoker)delegate
                //{
                //    Alarm_lbl.Left = Width / 2 - Alarm_lbl.Width / 2;
                //});
                Rdy_lbl.Invoke((MethodInvoker)delegate
                {
                    Rdy_lbl.Left = Width / 2 - Rdy_lbl.Width / 2;
                });





                switch (stateForm)
                {
                    case WorkState.Idle:

                        try
                        {
                            Alarm_panel.Invoke((MethodInvoker)delegate
                            {
                                Alarm_panel.BackColor = Color.White;
                            });
                            Alarm_lbl.Invoke((MethodInvoker)delegate
                            {
                                Alarm_lbl.ForeColor = Color.Black;
                                Alarm_lbl.Text = "Не подключен";
                        //        Alarm_lbl.Left = this.Width / 2 - Alarm_lbl.Width / 2;
                                Alarm_lbl.Left = Width / 2 - Alarm_lbl.Width / 2;
                            });

                            Rdy_lbl.Invoke((MethodInvoker)delegate
                            {
                                Rdy_lbl.Text = "Не готов";
                            });

                            Rdy_panel.Invoke((MethodInvoker)delegate
                            {
                                Rdy_panel.BackColor = System.Drawing.SystemColors.ActiveBorder;
                            });
                        }
                        catch (Exception ex)
                        { }



                        break;
                    case WorkState.Update:



                        if (state == State.New)
                        {

                            if (is_connected  && ((Read_reg & (1 << 5)) == 32))
                            {
                                Rdy_lbl.Invoke((MethodInvoker)delegate
                                {
                                    Rdy_lbl.Text = "Готов";
                                });

                                Rdy_panel.Invoke((MethodInvoker)delegate
                                {
                                    Rdy_panel.BackColor = Color.Green;
                                });

                                Check_alarm();





                            }
                            else if(is_connected)
                            {
                                Alarm_panel.Invoke((MethodInvoker)delegate
                                {
                                    Alarm_panel.BackColor = Color.White;
                                });
                                Alarm_lbl.Invoke((MethodInvoker)delegate
                                {
                                    Alarm_lbl.ForeColor = Color.Black;
                                    Alarm_lbl.Text = "Не готов";
                                    Alarm_lbl.Left = this.Width / 2 - Alarm_lbl.Width / 2;
                                });

                                Rdy_lbl.Invoke((MethodInvoker)delegate
                                {
                                    Rdy_lbl.Text = "Не готов";
                                });

                                Rdy_panel.Invoke((MethodInvoker)delegate
                                {
                                    Rdy_panel.BackColor = System.Drawing.SystemColors.ActiveBorder;
                                });
                            }
                            else
                            {
                                Alarm_panel.Invoke((MethodInvoker)delegate
                                {
                                    Alarm_panel.BackColor = Color.White;
                                });
                                Alarm_lbl.Invoke((MethodInvoker)delegate
                                {
                                    Alarm_lbl.ForeColor = Color.Black;
                                    Alarm_lbl.Text = "Не подключен";
                                    Alarm_lbl.Left = this.Width / 2 - Alarm_lbl.Width / 2;
                                });

                                Rdy_lbl.Invoke((MethodInvoker)delegate
                                {
                                    Rdy_lbl.Text = "Не готов";
                                });

                                Rdy_panel.Invoke((MethodInvoker)delegate
                                {
                                    Rdy_panel.BackColor = System.Drawing.SystemColors.ActiveBorder;
                                });
                            }


                            state = State.Read;
                        }
                        break;


                }
            }
        }




        public void Check_alarm()
        {
            if ((Read_reg & (1 << 0)) == 1)
                stt = 2;
            else if ((Read_reg & (1 << 1)) == 2)
                stt = 4;
            else if ((Read_reg & (1 << 2)) == 4)
                stt = 1;
            else if ((Read_reg & (1 << 3)) == 8)
                stt = 3;
            else
                stt = 0;



                Chec(stt);
        }

        private void M_bgwModbusUpdater_ProgressChanged(object sender, ProgressChangedEventArgs e)
        {
          //  throw new NotImplementedException();
        }

        public int sleeptime = 100;
        private void M_bgwModbusUpdater_DoWork(object sender, DoWorkEventArgs e)
        {
            {
                while (true)
                {
                    if (m_bgwModbusUpdater.CancellationPending)
                    {
                        e.Cancel = true;
                        break;
                    }

                    switch (stateRegs)
                    {
                        case WorkState.Idle:
                            break;
                        case WorkState.Update:
                            ModbusUpdate();
                            m_bgwModbusUpdater.ReportProgress(1);


                            // this.Text = (string.IsNullOrEmpty(m_lastError)) ? "Form1" : ("Form1... Error: " + m_state);

                            break;
                        default:
                            break;
                    }

                    Thread.Sleep(sleeptime);
                }
            }
        }

        private void Form_Update_ProgressChanged(object sender, ProgressChangedEventArgs e)
        {
          //  throw new NotImplementedException();
        }

        public static IModbusMaster m_mbMaster;
        readonly BackgroundWorker m_bgwModbusUpdater;
        readonly BackgroundWorker Form_Update;
        private void ConnectMenu_Click(object sender, EventArgs e)
        {
            string[] av_port = System.IO.Ports.SerialPort.GetPortNames();
            ConnectMenu.DropDownItems.Clear();

            ConnectMenu.DropDownItems.Add("Отключить");
            ConnectMenu.DropDownItems.Add("Настройка");
            foreach (string port in av_port)
                ConnectMenu.DropDownItems.Add(port);
        }
        object m_syncObject = new object();
        public enum State
        {
            Read,
            Write,
            New,
            Set
        }
        public State state { get; set; }
        ushort error_cnt = 0;
        ushort mb_ok = 0;
        ushort mb_tresh = 100;
        ushort[] rdBuf;
        byte slaveaddr = 1;
        ushort read_start_addr = 11;
        ushort read_regs_number = 40;

        int date_ind = 0;
        int id_ind = 25;
        int src_ind = 3;
        int md5_ind = 31;


        

        private bool ModbusUpdate()
        {
            try
            {
                lock (m_syncObject)
                {
                    switch (state)
                    {
                        case State.Read:
                            try
                            {
                                rdBuf = m_mbMaster.ReadHoldingRegisters(slaveaddr, read_start_addr, read_regs_number);
                                state = State.New;

                                Array.Copy(rdBuf, date_ind, date_regs, 0, 3);   //++
                                Array.Copy(rdBuf, id_ind, ID, 0, 6);            //++
                                Array.Copy(rdBuf, src_ind, src_num_regs, 0, 2); //++
                                Array.Copy(rdBuf, md5_ind, md5_regs, 0, 8);     //++
                                Read_reg = rdBuf[19];   //++
                                ver_reg = rdBuf[39];    //++

                                Status_lbl.Invoke((MethodInvoker)delegate
                                {
                                    Status_lbl.Text = "Подключен";
                                });

                                mb_ok++;
                                is_connected = true;
                                if (mb_ok > mb_tresh)
                                {
                                    mb_ok = 0;
                                    error_cnt = 0;
                                }

                            }
                            catch(Exception ex)
                            {
                                string strcgc = ex.Message;
                                m_lastError = "Ошибка чтения";
                                error_cnt++;


                                Status_lbl.Invoke((MethodInvoker)delegate
                                {
                                    Status_lbl.Text = m_lastError + " " + error_cnt.ToString();
                                });

                                if(error_cnt > mb_tresh)
                                {
                                    is_connected = false;
                                }
                                state = State.New;
                                return false;
                            }
                            break;
                        case State.Write:
                            try
                            {
                                state = State.Set;
                            }
                            catch (Exception ex)
                            {
                                return false;
                            }
                            break;
                        case State.New:
                            break;
                        case State.Set:
                            state = State.Read;
                            break;
                    }


                }

                return true;
            }
            catch (Exception ex)
            {
                m_lastError = ex.Message;
                return false;
            }
        }




        public enum WorkState
        {
            Idle,
            Update
        }
        bool is_connected = false;
        bool is_open = false;
        public bool Csisopen { get; set; }
        private WorkState stateForm = new WorkState();
        public WorkState stateRegs { get; set; }
        string m_lastError;
        public void COM_set(int br, int db, int par, int sb, Int32 ad)
        {

            switch (br)
            {
                case 0:
                    COM.BaudRate = 9600;
                    break;
                case 1:
                    COM.BaudRate = 115200;
                    break;
            }

            switch (par)
            {
                case 0:
                    COM.Parity = System.IO.Ports.Parity.None;
                    break;
                case 1:
                    COM.Parity = System.IO.Ports.Parity.Even;
                    break;
                case 2:
                    COM.Parity = System.IO.Ports.Parity.Odd;
                    break;
            }

            switch (sb)
            {
                case 0:
                    COM.StopBits = System.IO.Ports.StopBits.One;
                    break;
                case 1:
                    COM.StopBits = System.IO.Ports.StopBits.One;
                    break;

            }


            //m_slaveAddr = (byte)ad;

            COM.DataBits = db + 7;

            slaveaddr = (byte)ad;
            //COM.Parity = par;
            //COM.StopBits = sb;


        }
        private void ConnectMenu_DropDownItemClicked(object sender, ToolStripItemClickedEventArgs e)
        {
            string item = e.ClickedItem.ToString(); //ToolStripItemClickedEventArgs.ClickedItem;
            if (item == "Отключить" && COM.IsOpen)
            {
                COM.Close();
                is_connected = false;
                stateForm = WorkState.Idle;
                // Connectbtn.Enabled = false;
                stateRegs = WorkState.Idle;

                //return;
            }
            else if (item == "Настройка")
            {
                if (!is_open)
                {
                    if (COM.IsOpen)
                        COM.Close();

                    stateRegs = WorkState.Idle;
                    stateForm = WorkState.Idle;



                    Form Cs = new WindowsFormsApplication1.COM_settings();

                    Cs.Show();
                    is_open = true;
                    Csisopen = true;


                }



            }
            else
            {

                if (COM.IsOpen)
                    COM.Close();

                COM.PortName = item;
                try
                {
                    COM.Open();
                    is_open = true;
                  //  is_connected = true;

                    stateForm = WorkState.Update;
                    stateRegs = WorkState.Update;
                }
                catch (Exception ex)
                {
                    m_lastError = ex.Message;
                }


            }
        }
   //     ushort date, src_num, ver;
   //     ushort[] md5;

        private void CHB_btn_CheckedChanged(object sender, EventArgs e)
        {
            if(CHB_btn.Checked)
            {
                TopMost = true;
            }
            else
            {
                TopMost = false;
            }
        }



        ushort[] id;
        private void toolStripMenuItem1_Click(object sender, EventArgs e)
        {
            if (is_connected)
            {
                Form Inf = new InfoForm(date_regs, src_num_regs, md5_regs, ID, ver_reg);
                Inf.Show();
            }
            else
            {
                MessageBox.Show("Не подключен");
            }
        }
    }
}
