using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;
using System.Windows.Forms.DataVisualization.Charting;
using System.Diagnostics;
using System.Threading;
using System.IO;



namespace Project_Main
{
    public partial class Form1 : Form
    {
        /* @brief: Private variables */

        SerialPort sPort;

        RECVLOG log;

        SENDER send;
        private delegate void ThreadDelegate();
        //private static System.Timers.Timer time;

        /* @brief: PACKET 구조체 값을 받을 일반 변수]
         * P_stx: PACKET 구조체의 STX 값을 임시 저장하는 일반 변수이다.
         * P_end: PACKET 구조체의 END 값을 임시 저장하는 일반 변수이다.
         * P_msgid: PACKET 구조체의 MSG_ID 값을 임시 저장하는 일반 변수이다.
         * P_Len: PACKET 구조체의 Length 값을 임시 저장하는 일반 변수이다.
        */

        /* @brief 시뮬레이션 Timer 변수
         *
         */
        //private Timer t = new Timer();
        private Random r = new Random();

        TextBox textBox = new TextBox();
        StreamWriter sw;
        /* @brief: ListViewItem 클래스의 새 인스턴스를 초기화.*/
        //ListViewItem lvi = new ListViewItem();

        UInt16[] packet_buff = new UInt16[4];

        byte[] payload_buff = new byte[28];
        byte[] rc_payload = new byte[28];
        byte[] motor_buff = new byte[28];


        float[] Dob_motor = new float[4];
        int[] Dob_rc = new int[7];

        int count_sum = 0;
        int pwm_count = 0;

// 데이터 송신에 필요한 vaariable
        string flo1_txt;
        string flo2_txt;
        int uin;

        byte[] strByte = new byte[12];
        byte[] intByte = new byte[12];

        byte[] flo1 = new byte[12];
        byte[] flo2 = new byte[12];
        byte[] Touin = new byte[12];
        //int value;

        /* Initial communication and execution */
        public Form1()
        {
            InitializeComponent();

            connectSetting();
            progressBarSet();
            Gyro_chartSetting();
            Motor_chartSetting();
            PWM_chartSetting();
            sendItem();

            btnConnect.Enabled = true;
            btnDisconnect.Enabled = false;
        }



        /* @brief: COMBOBOX 인덱스 변경 이벤트
         * @param:
         *  sender: The object sending the event 
         *  e: Parameters used by event handlers
         * @retral: None
         */
        private void comPort_SelectedIndexChanged(object sender, EventArgs e)
        {
            ComboBox pt = sender as ComboBox;
            sPort = new SerialPort(pt.SelectedItem.ToString()); //문자열로 Port 반환
        }
        private void comBoudrate_SelectedIndexChanged(object sender, EventArgs e)
        {
            ComboBox br = sender as ComboBox;
            sPort.BaudRate = Convert.ToInt32(br.SelectedItem); //선택 시 BOUDRATE 변경
        }

        /* @brief: CONNECT, DISCONNECT 버튼 클릭 이벤트
         * @param:
         *  sender: The object sending the event 
         *  e: Parameters used by event handlers
         * @retral: None
         */
        private void btnConnect_Click(object sender, EventArgs e) // connect버튼 누르면 통신 시작
        {
            if (sPort.PortName == "")
            {
                MessageBox.Show("Select Serial Port Available First", "Warning");
                return;
            }

            sPort.Open();
            textBox1.Text += DateTime.Now.ToString() + " 연결이 성공적으로 되었습니다" + "\r\n";
            lblConnection.Text = "Connection Time : " + DateTime.Now.ToString(); //통신 시간 설정

            sPort.DataReceived += new SerialDataReceivedEventHandler(SPort_DataReceived); //SPort_DataReceived로 이동하여 데이터 읽기 시작
            //sPort.DataReceived += SPort_DataReceived;

            btnConnect.Enabled = false;
            btnDisconnect.Enabled = true;
        }

        private void btnDisconnect_Click(object sender, EventArgs e) // Disconnect버튼 누르면 통신 중단
        {
            sPort.DiscardOutBuffer();
            sPort.Close();
            
            sPort = null;

            textBox1.Text += DateTime.Now.ToString() + " 수고하셨습니다" + "\r\n";
            btnConnect.Enabled = true;
            btnDisconnect.Enabled = false;
        }

        /* @brief: Data Receive Function
         * @param:
         *  sender: The object sending the event 
         *  e: Provide data on events
         * @retral: None
         */
        public void SPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            //chart2.BeginInvoke(new EventHandler(SerialReceived));  //메인 쓰레드와 수신 쓰레드의 충돌 방지를 위해 Invoke 사용. SerialReceived로 이동하여 추가 작업 실행.
            UInt16 msgID = STX_proc();
            if(msgID == 0)
            {
                motor_buff = SerialReceived();
            }
            else if (msgID == 1)
            {
                Dob_rc = rc_SerialReceived();
            }
            else
            {
                return;
            }

            packet_buff[3] = (UInt16)sPort.ReadByte();
            if (packet_buff[3] == 0xEE)
            {
                if (msgID == 0)
                {
                    //ThreadDelegate Data_prog = new ThreadDelegate(motor_ReadData);
                    //this.BeginInvoke(Data_prog);
                    this.BeginInvoke((new Action(delegate { motor_ReadData(Dob_motor); })));
                }
                else if (msgID == 1)
                {
                    this.BeginInvoke((new Action(delegate { rc_ReadData(Dob_rc); })));
                }
                else
                {
                    return;
                }
            }
            else
            {
                return;
            }

        }

        /* @brief: Data Processing
         * @param:
         *  sender: The object sending the event 
         *  e: Paramaters used by event handler
         * @retral: None
         */

        private UInt16 STX_proc()
        {
            try
            {
                do
                {
                    packet_buff[0] = (UInt16)sPort.ReadByte();
                } while (packet_buff[0] != 0xFF);

                for (int j = 1; j < 3; j++)
                {
                    packet_buff[j] = (UInt16)sPort.ReadByte();
                }
            }
            catch(Exception x)
            {
                MessageBox.Show(x.ToString());
            }
       
            return packet_buff[2];
        }

        private byte[] SerialReceived()  //여기에서 수신 데이터를 사용자의 용도에 따라 처리한다.
        {
            try
            {
                // var timer = Stopwatch.StartNew(); 
                for (int i = 0; i < 7; i++)
                {
                    payload_buff[(i * 4) + 3] = (byte)sPort.ReadByte();
                    payload_buff[(i * 4) + 2] = (byte)sPort.ReadByte();
                    payload_buff[(i * 4) + 1] = (byte)sPort.ReadByte();
                    payload_buff[(i * 4)] = (byte)sPort.ReadByte();
                }
            }catch(Exception x)
            {
                MessageBox.Show(x.ToString());
            }

            return payload_buff;
            //timer.Stop();
            //btn_Time.Text = timer.ElapsedMilliseconds + "s";
        }
        private int[] rc_SerialReceived()  //여기에서 수신 데이터를 사용자의 용도에 따라 처리한다.
        {
            int[] a = new int[7];

            try
            {
                // var timer = Stopwatch.StartNew();
                for (int i = 0; i < 7; i++)
                {
                    rc_payload[(i * 4)] = (byte)sPort.ReadByte();
                    rc_payload[(i * 4) + 1] = (byte)sPort.ReadByte();
                    rc_payload[(i * 4) + 2] = (byte)sPort.ReadByte();
                    rc_payload[(i * 4) + 3] = (byte)sPort.ReadByte();
                }

                a[0] = ((rc_payload[0] << 24) | (rc_payload[1] << 16) | (rc_payload[2] << 8) | (rc_payload[3]));
                a[1] = ((rc_payload[4] << 24) | (rc_payload[5] << 16) | (rc_payload[6] << 8) | (rc_payload[7]));
                a[2] = ((rc_payload[8] << 24) | (rc_payload[9] << 16) | (rc_payload[10] << 8) | (rc_payload[11]));
                a[3] = ((rc_payload[12] << 24) | (rc_payload[13] << 16) | (rc_payload[14] << 8) | (rc_payload[15]));
                a[4] = ((rc_payload[16] << 24) | (rc_payload[17] << 16) | (rc_payload[18] << 8) | (rc_payload[19]));
                a[5] = ((rc_payload[20] << 24) | (rc_payload[21] << 16) | (rc_payload[22] << 8) | (rc_payload[23]));
                a[6] = ((rc_payload[24] << 24) | (rc_payload[25] << 16) | (rc_payload[26] << 8) | (rc_payload[27]));

            }
            catch (Exception x)
            {
                MessageBox.Show(x.ToString());
            }

            return a;
            //timer.Stop();
            //btn_Time.Text = timer.ElapsedMilliseconds + "s";
        }

        /* @brief: ConnectSetting */
        private void connectSetting()
        {
            foreach (var port in SerialPort.GetPortNames()) //내가 사용할 수 있는 모든 port를 열어준다.
            {
                comPort.Items.Add(port);
            }
            comPort.Text = "Select Port";

            comBoudrate.Text = "Select Boud Rate";
            comBoudrate.Items.Add("9600");       //문자열로 BOUDRATE 설정
            comBoudrate.Items.Add("115200");
        }

        /* @brief: ProgressBarSetting */
        private void progressBarSet()
        {
            proThrottle.Minimum = 342; //디폴트값 342
            proPitch.Minimum = 342;
            proRoll.Minimum = 342;
            proYaw.Minimum = 342;

            proThrottle.Maximum = 1706;
            proPitch.Maximum = 1706;
            proRoll.Maximum = 1706;
            proYaw.Maximum = 1706;
        }

        /* @brief: Gyro, Motor, PWM chartsetting */
        private void Gyro_chartSetting()
        {
            chart1.ChartAreas.Clear();
            chart1.Series.Clear();

            chart1.ChartAreas.Add("ChartArea1");

            chart1.ChartAreas["ChartArea1"].AxisX.Minimum = 0;
            chart1.ChartAreas["ChartArea1"].AxisX.Maximum = 200;
            chart1.ChartAreas["ChartArea1"].AxisX.Interval = 50;

            chart1.ChartAreas["ChartArea1"].AxisY.Minimum = 0;
            chart1.ChartAreas["ChartArea1"].AxisY.Maximum = 400;
            chart1.ChartAreas["ChartArea1"].AxisY.Interval = 100;
            chart1.ChartAreas["ChartArea1"].BackColor = Color.White;

            chart1.ChartAreas["ChartArea1"].CursorX.AutoScroll = true;

            chart1.ChartAreas["ChartArea1"].AxisX.ScaleView.Zoomable = true;
            chart1.ChartAreas["ChartArea1"].AxisX.ScrollBar.ButtonStyle = ScrollBarButtonStyles.SmallScroll;
            chart1.ChartAreas["ChartArea1"].AxisX.ScrollBar.ButtonColor = Color.LightSteelBlue;

            chart1.Series.Add("Pitch");
            chart1.Series["Pitch"].ChartType = SeriesChartType.Spline;
            chart1.Series["Pitch"].Color = Color.Red;
            chart1.Series["Pitch"].BorderWidth = 1;

            chart1.Series.Add("Roll");
            chart1.Series["Roll"].ChartType = SeriesChartType.Spline;
            chart1.Series["Roll"].Color = Color.Black;
            chart1.Series["Roll"].BorderWidth = 1;

            chart1.Series.Add("Yaw");
            chart1.Series["Yaw"].ChartType = SeriesChartType.Spline;
            chart1.Series["Yaw"].Color = Color.Blue;
            chart1.Series["Yaw"].BorderWidth = 1;

        }
        private void Motor_chartSetting()
        {
            chart2.ChartAreas.Clear();
            chart2.Series.Clear();

            chart2.ChartAreas.Add("ChartArea2");
            chart2.ChartAreas["ChartArea2"].AxisX.Minimum = 0;
            chart2.ChartAreas["ChartArea2"].AxisX.Maximum = 200;
            chart2.ChartAreas["ChartArea2"].AxisX.Interval = 50;
            chart2.ChartAreas["ChartArea2"].AxisX.MajorGrid.LineColor = Color.Black;
            chart2.ChartAreas["ChartArea2"].AxisX.MajorGrid.LineDashStyle = ChartDashStyle.Dash;

            chart2.ChartAreas["ChartArea2"].AxisY.Minimum = 0;
            chart2.ChartAreas["ChartArea2"].AxisY.Maximum = 8000;
            chart2.ChartAreas["ChartArea2"].AxisY.Interval = 2000;
            chart2.ChartAreas["ChartArea2"].AxisY.MajorGrid.LineColor = Color.Black;
            chart2.ChartAreas["ChartArea2"].AxisY.MajorGrid.LineDashStyle = ChartDashStyle.Dash;
            chart2.ChartAreas["ChartArea2"].BackColor = Color.White;
            chart2.ChartAreas["ChartArea2"].CursorX.AutoScroll = true;

            chart2.ChartAreas["ChartArea2"].AxisX.ScaleView.Zoomable = true;
            chart2.ChartAreas["ChartArea2"].AxisX.ScrollBar.ButtonStyle = ScrollBarButtonStyles.SmallScroll;
            chart2.ChartAreas["ChartArea2"].AxisX.ScrollBar.ButtonColor = Color.LightSteelBlue;

            chart2.Series.Add("Encoder");
            chart2.Series["Encoder"].ChartType = SeriesChartType.Spline;
            chart2.Series["Encoder"].Color = Color.Red;
            chart2.Series["Encoder"].BorderWidth = 1;

            chart2.Series.Add("RPM");
            chart2.Series["RPM"].ChartType = SeriesChartType.Spline;
            chart2.Series["RPM"].Color = Color.Black;
            chart2.Series["RPM"].BorderWidth = 1;

            chart2.Series.Add("Setpoint");
            chart2.Series["Setpoint"].ChartType = SeriesChartType.Spline;
            chart2.Series["Setpoint"].Color = Color.Blue;
            chart2.Series["Setpoint"].BorderWidth = 1;
        }
        private void PWM_chartSetting()
        {
            chart3.ChartAreas.Clear();
            chart3.Series.Clear();

            chart3.ChartAreas.Add("ChartArea3");
            chart3.ChartAreas["ChartArea3"].AxisX.Minimum = 0;
            chart3.ChartAreas["ChartArea3"].AxisX.Maximum = 200;
            chart3.ChartAreas["ChartArea3"].AxisX.Interval = 50;
            chart3.ChartAreas["ChartArea3"].AxisX.MajorGrid.LineColor = Color.Black;
            chart3.ChartAreas["ChartArea3"].AxisX.MajorGrid.LineDashStyle = ChartDashStyle.Dash;

            chart3.ChartAreas["ChartArea3"].AxisY.Minimum =1000;
            chart3.ChartAreas["ChartArea3"].AxisY.Maximum = 2000; //테스트값
            chart3.ChartAreas["ChartArea3"].AxisY.Interval = 500;
            chart3.ChartAreas["ChartArea3"].AxisY.MajorGrid.LineColor = Color.Black;
            chart3.ChartAreas["ChartArea3"].AxisY.MajorGrid.LineDashStyle = ChartDashStyle.Dash;
            chart3.ChartAreas["ChartArea3"].BackColor = Color.White;
            chart3.ChartAreas["ChartArea3"].CursorX.AutoScroll = true;

            chart3.ChartAreas["ChartArea3"].AxisX.ScaleView.Zoomable = true;
            chart3.ChartAreas["ChartArea3"].AxisX.ScrollBar.ButtonStyle = ScrollBarButtonStyles.SmallScroll;
            chart3.ChartAreas["ChartArea3"].AxisX.ScrollBar.ButtonColor = Color.LightSteelBlue;

            chart3.Series.Add("PWM");
            chart3.Series["PWM"].ChartType = SeriesChartType.Spline;
            chart3.Series["PWM"].Color = Color.Blue;
            chart3.Series["PWM"].BorderWidth = 1;
        }

        private void log_ReadData()
        {
            log.logMsg = Convert.ToChar(sPort.ReadByte());
        }

        /* brief: MOTOR READ FUNCTION */
        private void motor_ReadData(float[] mot)
        {
            mot[0] = BitConverter.ToSingle(motor_buff, 0);
            mot[1] = BitConverter.ToSingle(motor_buff, 4);
            mot[2] = BitConverter.ToSingle(motor_buff, 8);
            mot[3] = BitConverter.ToSingle(motor_buff, 12);

            if (chEncoder.Checked == true)
            {                
                chart2.ChartAreas["ChartArea2"].AxisY.Maximum = 400;
                chart2.ChartAreas["ChartArea2"].AxisY.Interval = 100;

                chart2.Series["Encoder"].Points.Add(mot[0]);
                count_sum = chart2.Series["Encoder"].Points.Count;

                chart2.ChartAreas["ChartArea2"].AxisX.Minimum = 0;
                chart2.ChartAreas["ChartArea2"].AxisX.Maximum = (count_sum >= 200) ? count_sum : 200;

                if (count_sum > 200)
                {
                    chart2.ChartAreas["ChartArea2"].AxisX.ScaleView.Zoom(count_sum - 200, count_sum);
                }
                else
                {
                    chart2.ChartAreas["ChartArea2"].AxisX.ScaleView.Zoom(0, 200);
                }
            }
            else
            {
                chart2.Series["Encoder"].Points.Clear();
            }

            if (chTime.Checked == true)
            {
                chart2.ChartAreas["ChartArea2"].AxisY.Maximum = 8000;
                chart2.ChartAreas["ChartArea2"].AxisY.Interval = 2000;

                chart2.Series["RPM"].Points.Add(mot[1]);

                count_sum = chart2.Series["RPM"].Points.Count;

                chart2.ChartAreas["ChartArea2"].AxisX.Minimum = 0;
                chart2.ChartAreas["ChartArea2"].AxisX.Maximum = (count_sum >= 200) ? count_sum : 200;

                if (count_sum > 200)
                {
                    chart2.ChartAreas["ChartArea2"].AxisX.ScaleView.Zoom(count_sum - 200, count_sum);
                    //chart2.ChartAreas["ChartArea2"].AxisX.Minimum = count_sum - (count_sum - 1);
                }
                else
                {
                    chart2.ChartAreas["ChartArea2"].AxisX.ScaleView.Zoom(0, 200);
                }
            }
            else
            {
                chart2.Series["RPM"].Points.Clear();
            }


            if (chAcceleration.Checked == true)
            {
                chart2.ChartAreas["ChartArea2"].AxisY.Maximum = 8000;
                chart2.ChartAreas["ChartArea2"].AxisY.Interval = 2000;

                chart2.Series["Setpoint"].Points.Add(mot[2]);

                count_sum = chart2.Series["Setpoint"].Points.Count;

                chart2.ChartAreas["ChartArea2"].AxisX.Minimum = 0;
                chart2.ChartAreas["ChartArea2"].AxisX.Maximum = (count_sum >= 200) ? count_sum : 200;

                if (count_sum > 200)
                {
                    chart2.ChartAreas["ChartArea2"].AxisX.ScaleView.Zoom(count_sum - 200, count_sum);
                }
                else
                {
                    chart2.ChartAreas["ChartArea2"].AxisX.ScaleView.Zoom(0, 200);
                }
            }
            else
            {
                chart2.Series["Setpoint"].Points.Clear();
            }

            //count가 200초과되면 count -1을 한다.]

            if (chPWM.Checked == true)
            {
                chart3.Series["PWM"].Points.Add(mot[3]);

                pwm_count = chart3.Series["PWM"].Points.Count;

                chart3.ChartAreas["ChartArea3"].AxisX.Minimum = 0;
                chart3.ChartAreas["ChartArea3"].AxisX.Maximum = (pwm_count >= 200) ? pwm_count : 200;

                if (pwm_count > 200)
                {                   
                    chart3.ChartAreas["ChartArea3"].AxisX.ScaleView.Zoom(pwm_count - 200, pwm_count);
                }
                else
                {
                    chart3.ChartAreas["ChartArea3"].AxisX.ScaleView.Zoom(0, 200);
                }
            }
            else
            {
                chart3.Series["PWM"].Points.Clear();
            }

            if(chListbox.Checked == true)
            {
                string item = (DateTime.Now.ToString() +"\t"+ mot[0] + "\t" + mot[1] + "\t" + mot[2] + "\t" + mot[3]);
                listBox1.Items.Add(item);
                listBox1.SelectedIndex = listBox1.Items.Count - 1;
            }
            else
            {
                listBox1.Items.Clear();
            }           
        }

        private void rc_ReadData(int[] r)
        {
            if (chProgressbar.Checked == true)
            {         
                if(r[0] > 0 && r[0] < 2000) proThrottle.Value = r[0];
                if (r[1] > 0 && r[1] < 2000) proPitch.Value  = r[2];
                if (r[2] > 0 && r[2] < 2000) proRoll.Value = r[1];
                if (r[3] > 0 && r[3] < 2000) proYaw.Value = r[3];
            }

            if (r[4] < 500)
            {
                aux1.Text = "OFF" + "\n" + r[4]; // 센터,폰트 맞추기
            }
            else if(r[4] > 500 && r[4] < 1300)
            {
                aux1.Text = "Speed" + "\n" + r[4];
            }
            else
            {
                aux1.Text = "Moment" + "\n" + r[4];
            }

            if (r[5] < 800)
            {
                aux2.Text = "OK" + "\n" + r[5];
            }
            else
            {
                aux2.Text = "Setting" + "\n" + r[5];
            }

            if(r[6] < 500)
            {
                aux3.Text = "CUT_OFF" + "\n" + r[6];
            }
            else if(r[6] > 500 && r[6] < 1300)
            {
                aux3.Text = "Arming 1" + "\n" + r[6];
            }
            else
            {
                aux3.Text = "Arming 2" + "\n" + r[6];
            }

        }

        private void button1_Click(object sender, EventArgs e)
        {
            chart2.Series["RPM"].Points.Clear();
            chart2.Series["Setpoint"].Points.Clear();
            chart2.Series["Encoder"].Points.Clear();
            chart3.Series["PWM"].Points.Clear();
        }

        private void saveToolStripMenuItem_Click(object sender, EventArgs e)
        {
            sw = new StreamWriter("Log.txt");
            int tcount = listBox1.Items.Count;

            for(int i = 0; i < tcount; i++)
            {
                listBox1.Items[i] += "\r\n";

                sw.Write(listBox1.Items[i]);
            }
            sw.Close();

            Process.Start("Log.txt");
        }

        private void loadCellToolStripMenuItem_Click_1(object sender, EventArgs e)
        {
            LoadCell_Form form2 = new LoadCell_Form();
            Form1 form1 = new Form1();
            //Application.ExitThread();
            //Environment.Exit(0);
            
            form2.ShowDialog();
            form1.Close();
            //form2.Owner = this;
            
            
        }

        private void sendItem()
        {
            comuInt.Text = "Select Data";
            comuInt.Items.Add("Speed_Gain");       //문자열로 BOUDRATE 설정
            comuInt.Items.Add("Moment_Gain");
            comuInt.Items.Add("LPF_Gain");
            comuInt.Items.Add("Amplitude_Gain");
        }

        private void btnSender_Click_1(object sender, EventArgs e)
        {
            flo1_txt = txtfloat1.Text;
            flo1 = StringToByte(flo1_txt);
            
            flo2_txt = txtfloat2.Text;
            flo2 = StringToByte(flo2_txt);

            sPort.Write(flo1, 0, 4);
            sPort.Write(flo2, 0, 4);
            sPort.Write(Touin, 0, 1);
        }

        private void comuInt_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (comuInt.Text == "Speed_Gain")
            {
                uin = 10;
            }
            else if (comuInt.Text == "Moment_Gain")
            {
                uin = 20;
            }
            else if (comuInt.Text == "LPF_Gain")
            {
                uin = 30;
            }
            else if (comuInt.Text == "Amplitude_Gain")
            {
                uin = 40;
            }
            else
            {
                return;
            }

            Touin = IntToByte(uin);

        }

        private byte[] StringToByte(string a)
        {
            //strByte = Encoding.UTF8.GetBytes(a);
            float fl = (float.Parse(a));
            strByte = BitConverter.GetBytes(fl);
            //return strByte;
            return strByte;
        }

        private byte[] IntToByte(int b)
        {
            intByte = BitConverter.GetBytes(b);

            return intByte;
        }


        // 시뮬레이션
        /*
                        private void startToolStripMenuItem_Click(object sender, EventArgs e)
                        {
                            t.Interval = 100;
                            t.Tick += T_Tick;
                            t.Start();
                            textBox1.Text = "시뮬레이션이 시작합니다.";
                        }


                private void T_Tick(object sender, EventArgs e)
                {
                    motor.ang = (ushort)(r.Next(400));
                    motor.acceleration = (ushort)(r.Next(300));
                    motor.time = (ushort)(r.Next(10));
                    motor.pwm = (ushort)(r.Next(5));

                    if (chEncoder.Checked == true)
                    {
                        chart2.Series["Encoder"].Points.Add(motor.ang);
                        count_sum = chart2.Series["Encoder"].Points.Count;

                        chart2.ChartAreas["ChartArea2"].AxisX.Minimum = 0;
                        chart2.ChartAreas["ChartArea2"].AxisX.Maximum = (count_sum >= 200) ? count_sum : 200;

                        if (count_sum > 200)
                        {
                            chart2.ChartAreas["ChartArea2"].AxisX.ScaleView.Zoom(count_sum - 200, count_sum);
                        }
                        else
                        {
                            chart2.ChartAreas["ChartArea2"].AxisX.ScaleView.Zoom(0, 200);
                        }

                    }

                    if (chTime.Checked == true)
                    {
                        chart2.Series["Time"].Points.Add(motor.time);

                        count_sum = chart2.Series["Time"].Points.Count;

                        chart2.ChartAreas["ChartArea2"].AxisX.Minimum = 0;
                        chart2.ChartAreas["ChartArea2"].AxisX.Maximum = (count_sum >= 200) ? count_sum : 200;

                        if (count_sum > 200)
                        {
                            chart2.ChartAreas["ChartArea2"].AxisX.ScaleView.Zoom(count_sum - 200, count_sum);
                        }
                        else
                        {
                            chart2.ChartAreas["ChartArea2"].AxisX.ScaleView.Zoom(0, 200);
                        }
                    }

                    if (chAcceleration.Checked == true)
                    {
                        chart2.Series["Acceleration"].Points.Add(motor.acceleration);

                        count_sum = chart2.Series["Acceleration"].Points.Count;

                        chart2.ChartAreas["ChartArea2"].AxisX.Minimum = 0;
                        chart2.ChartAreas["ChartArea2"].AxisX.Maximum = (count_sum >= 200) ? count_sum : 200;

                        if (count_sum > 200)
                        {
                            chart2.ChartAreas["ChartArea2"].AxisX.ScaleView.Zoom(count_sum - 200, count_sum);
                        }
                        else
                        {
                            chart2.ChartAreas["ChartArea2"].AxisX.ScaleView.Zoom(0, 200);
                        }
                    }

                    if (chPWM.Checked == true)
                    {
                        chart3.Series["PWM"].Points.Add(motor.pwm);

                        pwm_count = chart3.Series["PWM"].Points.Count;

                        chart3.ChartAreas["ChartArea3"].AxisX.Minimum = 0;
                        chart3.ChartAreas["ChartArea3"].AxisX.Maximum = (pwm_count >= 200) ? pwm_count : 200;

                        if (pwm_count > 200)
                        {
                            chart3.ChartAreas["ChartArea3"].AxisX.ScaleView.Zoom(pwm_count - 200, pwm_count);
                        }
                        else
                        {
                            chart3.ChartAreas["ChartArea3"].AxisX.ScaleView.Zoom(0, 200);
                        }
                    }
                    //chart2.Series["Acceleration"].Points.Add(motor.acceleration);
                    // chart2.Series["Time"].Points.Add(motor.time);


                    //chart3.Series["PWM"].Points.Add(motor.pwm);



                      motor.RPM = (ushort)(r.Next(1500));

                    string item = DateTime.Now.ToString() + "\t" + motor.RPM;
                    listBox1.Items.Add(item);
                    listBox1.SelectedIndex = listBox1.Items.Count - 1;

                    rc.throttle = (ushort)(r.Next(1700));
                    rc.roll = (ushort)(r.Next(1700));
                    rc.pitch = (ushort)(r.Next(1700));
                    rc.yaw = (ushort)(r.Next(1700));

                    proThrottle.Value = rc.throttle;
                    proPitch.Value = rc.pitch;
                    proRoll.Value = rc.roll;
                    proYaw.Value = rc.yaw;

                    //rc.aux1 = (ushort)(r.Next(1000));
                    //rc.aux2 = (ushort)(r.Next(1000));
                    //rc.aux3 = (ushort)(r.Next(1000));
                }
                /*
                        private void endToolStripMenuItem_Click(object sender, EventArgs e)
                        {
                            t.Stop();
                            textBox1.Text = "시뮬레이션이 종료합니다.";
                        }
                */
    }
}