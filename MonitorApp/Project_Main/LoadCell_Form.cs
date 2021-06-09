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
    public partial class LoadCell_Form : Form
    {
        byte[] rc = new byte[16];
        byte[] load = new byte[16];

        int cell_count = 0;
        UInt16 ch;

        SerialPort port;
        public LoadCell_Form()
        {
            InitializeComponent();

            connectSetting();
            Cell_chartSetting();
        }
        private void connectSetting()
        {
            foreach (var port in SerialPort.GetPortNames()) //내가 사용할 수 있는 모든 port를 열어준다.
            {
                comport.Items.Add(port);
            }
            comport.Text = "Select Port";

            comboud.Text = "Select Boud Rate";
            comboud.Items.Add("9600");       //문자열로 BOUDRATE 설정
            comboud.Items.Add("115200");
        }

        private void comport_SelectedIndexChanged(object sender, EventArgs e)
        {
            ComboBox p= sender as ComboBox;
            port = new SerialPort(p.SelectedItem.ToString()); //문자열로 Port 반환
        }

        private void comboud_SelectedIndexChanged(object sender, EventArgs e)
        {
            ComboBox b = sender as ComboBox;
            port.BaudRate = Convert.ToInt32(b.SelectedItem);
        }

        private void btconnect_Click(object sender, EventArgs e)
        {
            if (port.PortName == "")
            {
                MessageBox.Show("Select Serial Port Available First", "Warning");
                return;
            }

            port.Open();

            port.DataReceived += new SerialDataReceivedEventHandler(Port_DataReceived); //SPort_DataReceived로 이동하여 데이터 읽기 시작
            //sPort.DataReceived += SPort_DataReceived;

            btconnect.Enabled = false;
            btdisconnect.Enabled = true;
        }

        private void btdisconnect_Click(object sender, EventArgs e)
        {
            port.Close();
            port = null;

            btconnect.Enabled = true;
            btdisconnect.Enabled = false;
        }

        private void Port_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            UInt16 stx = STX();

            if(stx == 0xFF)
            {
                load = Data();
                this.BeginInvoke((new Action(delegate { showData(load); })));
            }
            
        }
        private UInt16 STX()
        {
            do
            {
                ch = (UInt16)port.ReadByte();
            } while (ch != 0xFF);

            return ch;
        }

        private byte[] Data() 
        {
            //int[] loadCell = new int[4];

            try
            {
                for (int i = 0; i < 4; i++)
                {
                    rc[(i * 4)] = (byte)(port.ReadByte());
                    rc[(i * 4) + 1] = (byte)(port.ReadByte());
                    rc[(i * 4) + 2] = (byte)(port.ReadByte());
                    rc[(i * 4) + 3] = (byte)(port.ReadByte());
                }
            }
            catch (Exception x)
            {
                MessageBox.Show(x.ToString());
            }

            return rc;
        }

        private void showData(byte[] cellData)
        {
             float[] cell = new float[4];
            
             cell[0] = BitConverter.ToSingle(rc, 0);
             cell[1] = BitConverter.ToSingle(rc, 4);
             cell[2] = BitConverter.ToSingle(rc, 8);
             cell[3] = BitConverter.ToSingle(rc, 12);
            
            if(button.Checked == true)
            {
                btfront.Text = "Front" + "\n" + cell[0];
                btrear.Text = "Rear" + "\n" + cell[1];
                btleft.Text = "Left" + "\n" + cell[2];
                btright.Text = "Right" + "\n" + cell[3];
            }
            
            if(listbox.Checked == true)
            {
                string item = cell[0] + "\t" + cell[1] + "\t" + cell[2] + "\t" + cell[3];
                listBox1.Items.Add(item);
                listBox1.SelectedIndex = listBox1.Items.Count - 1;
            
            }

            if (chgraph.Checked == true)
            {
                chart1.Series["Front"].Points.Add(cell[0]);
                chart1.Series["Rear"].Points.Add(cell[1]);
                chart1.Series["Left"].Points.Add(cell[2]);
                chart1.Series["Right"].Points.Add(cell[3]);

                cell_count = chart1.Series["Front"].Points.Count;

                chart1.ChartAreas["ChartArea1"].AxisX.Minimum = 0;
                chart1.ChartAreas["ChartArea1"].AxisX.Maximum = (cell_count >= 200) ? cell_count : 200;

                if (cell_count > 200)
                {
                    chart1.ChartAreas["ChartArea1"].AxisX.ScaleView.Zoom(cell_count - 200, cell_count);
                }
                else
                {
                    chart1.ChartAreas["ChartArea1"].AxisX.ScaleView.Zoom(0, 200);
                }

                if(cell_count > 2000)
                {
                    chart1.Series["Front"].Points.RemoveAt(0);
                    chart1.Series["Rear"].Points.RemoveAt(0);
                    chart1.Series["Left"].Points.RemoveAt(0);
                    chart1.Series["Right"].Points.RemoveAt(0);
                }

            }
            else
            {
                chart1.Series["Front"].Points.Clear();
                chart1.Series["Rear"].Points.Clear();
                chart1.Series["Left"].Points.Clear();
                chart1.Series["Right"].Points.Clear();
            }

        }

        private void Cell_chartSetting()
        {
            chart1.ChartAreas.Clear();
            chart1.Series.Clear();

            chart1.ChartAreas.Add("ChartArea1");

            chart1.ChartAreas["ChartArea1"].AxisX.Minimum = 0;
            chart1.ChartAreas["ChartArea1"].AxisX.Maximum = 200;
            chart1.ChartAreas["ChartArea1"].AxisX.Interval = 50;

            chart1.ChartAreas["ChartArea1"].AxisY.Minimum = 81000;
            chart1.ChartAreas["ChartArea1"].AxisY.Maximum = 81500;
            chart1.ChartAreas["ChartArea1"].AxisY.Interval = 100;
            chart1.ChartAreas["ChartArea1"].BackColor = Color.White;

            chart1.ChartAreas["ChartArea1"].CursorX.AutoScroll = true;

            chart1.ChartAreas["ChartArea1"].AxisX.ScaleView.Zoomable = true;
            chart1.ChartAreas["ChartArea1"].AxisX.ScrollBar.ButtonStyle = ScrollBarButtonStyles.SmallScroll;
            chart1.ChartAreas["ChartArea1"].AxisX.ScrollBar.ButtonColor = Color.LightSteelBlue;

            chart1.Series.Add("Front");
            chart1.Series["Front"].ChartType = SeriesChartType.Spline;
            chart1.Series["Front"].Color = Color.Red;
            chart1.Series["Front"].BorderWidth = 1;

            chart1.Series.Add("Rear");
            chart1.Series["Rear"].ChartType = SeriesChartType.Spline;
            chart1.Series["Rear"].Color = Color.Black;
            chart1.Series["Rear"].BorderWidth = 1;

            chart1.Series.Add("Left");
            chart1.Series["Left"].ChartType = SeriesChartType.Spline;
            chart1.Series["Left"].Color = Color.Blue;
            chart1.Series["Left"].BorderWidth = 1;

            chart1.Series.Add("Right");
            chart1.Series["Right"].ChartType = SeriesChartType.Spline;
            chart1.Series["Right"].Color = Color.Brown;
            chart1.Series["Right"].BorderWidth = 1;

        }
    }
}
