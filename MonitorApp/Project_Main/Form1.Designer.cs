
namespace Project_Main
{
    partial class Form1
    {
        /// <summary>
        /// 필수 디자이너 변수입니다.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// 사용 중인 모든 리소스를 정리합니다.
        /// </summary>
        /// <param name="disposing">관리되는 리소스를 삭제해야 하면 true이고, 그렇지 않으면 false입니다.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form 디자이너에서 생성한 코드

        /// <summary>
        /// 디자이너 지원에 필요한 메서드입니다. 
        /// 이 메서드의 내용을 코드 편집기로 수정하지 마세요.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            System.Windows.Forms.DataVisualization.Charting.ChartArea chartArea1 = new System.Windows.Forms.DataVisualization.Charting.ChartArea();
            System.Windows.Forms.DataVisualization.Charting.Legend legend1 = new System.Windows.Forms.DataVisualization.Charting.Legend();
            System.Windows.Forms.DataVisualization.Charting.Series series1 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Series series2 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Series series3 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Title title1 = new System.Windows.Forms.DataVisualization.Charting.Title();
            System.Windows.Forms.DataVisualization.Charting.ChartArea chartArea2 = new System.Windows.Forms.DataVisualization.Charting.ChartArea();
            System.Windows.Forms.DataVisualization.Charting.Legend legend2 = new System.Windows.Forms.DataVisualization.Charting.Legend();
            System.Windows.Forms.DataVisualization.Charting.Series series4 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Series series5 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Series series6 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Title title2 = new System.Windows.Forms.DataVisualization.Charting.Title();
            System.Windows.Forms.DataVisualization.Charting.ChartArea chartArea3 = new System.Windows.Forms.DataVisualization.Charting.ChartArea();
            System.Windows.Forms.DataVisualization.Charting.Legend legend3 = new System.Windows.Forms.DataVisualization.Charting.Legend();
            System.Windows.Forms.DataVisualization.Charting.Series series7 = new System.Windows.Forms.DataVisualization.Charting.Series();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.btnDisconnect = new System.Windows.Forms.Button();
            this.comPort = new System.Windows.Forms.ComboBox();
            this.comBoudrate = new System.Windows.Forms.ComboBox();
            this.btnConnect = new System.Windows.Forms.Button();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.chListbox = new System.Windows.Forms.CheckBox();
            this.chProgressbar = new System.Windows.Forms.CheckBox();
            this.chYaw = new System.Windows.Forms.CheckBox();
            this.chPitch = new System.Windows.Forms.CheckBox();
            this.chRoll = new System.Windows.Forms.CheckBox();
            this.chPWM = new System.Windows.Forms.CheckBox();
            this.chEncoder = new System.Windows.Forms.CheckBox();
            this.menuStrip1 = new System.Windows.Forms.MenuStrip();
            this.fileToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.saveToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.exitToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.loadCellToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.simulationToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.startToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.endToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.label7 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.proYaw = new System.Windows.Forms.ProgressBar();
            this.proRoll = new System.Windows.Forms.ProgressBar();
            this.proPitch = new System.Windows.Forms.ProgressBar();
            this.proThrottle = new System.Windows.Forms.ProgressBar();
            this.chart1 = new System.Windows.Forms.DataVisualization.Charting.Chart();
            this.serialPort1 = new System.IO.Ports.SerialPort(this.components);
            this.textBox1 = new System.Windows.Forms.TextBox();
            this.lblConnection = new System.Windows.Forms.Label();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.chAcceleration = new System.Windows.Forms.CheckBox();
            this.chTime = new System.Windows.Forms.CheckBox();
            this.groupBox5 = new System.Windows.Forms.GroupBox();
            this.AUX = new System.Windows.Forms.GroupBox();
            this.aux3 = new System.Windows.Forms.Button();
            this.aux2 = new System.Windows.Forms.Button();
            this.aux1 = new System.Windows.Forms.Button();
            this.chart2 = new System.Windows.Forms.DataVisualization.Charting.Chart();
            this.chart3 = new System.Windows.Forms.DataVisualization.Charting.Chart();
            this.listBox1 = new System.Windows.Forms.ListBox();
            this.groupBox6 = new System.Windows.Forms.GroupBox();
            this.button1 = new System.Windows.Forms.Button();
            this.groupBox7 = new System.Windows.Forms.GroupBox();
            this.label9 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.label1 = new System.Windows.Forms.Label();
            this.btnSender = new System.Windows.Forms.Button();
            this.comuInt = new System.Windows.Forms.ComboBox();
            this.txtfloat2 = new System.Windows.Forms.TextBox();
            this.txtfloat1 = new System.Windows.Forms.TextBox();
            this.groupBox1.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.menuStrip1.SuspendLayout();
            this.groupBox3.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.chart1)).BeginInit();
            this.groupBox4.SuspendLayout();
            this.AUX.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.chart2)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.chart3)).BeginInit();
            this.groupBox6.SuspendLayout();
            this.groupBox7.SuspendLayout();
            this.SuspendLayout();
            // 
            // groupBox1
            // 
            this.groupBox1.BackColor = System.Drawing.SystemColors.ControlDarkDark;
            this.groupBox1.Controls.Add(this.btnDisconnect);
            this.groupBox1.Controls.Add(this.comPort);
            this.groupBox1.Controls.Add(this.comBoudrate);
            this.groupBox1.Controls.Add(this.btnConnect);
            this.groupBox1.Cursor = System.Windows.Forms.Cursors.Hand;
            this.groupBox1.Font = new System.Drawing.Font("굴림", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.groupBox1.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.groupBox1.Location = new System.Drawing.Point(1124, 41);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(204, 241);
            this.groupBox1.TabIndex = 2;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Setting";
            // 
            // btnDisconnect
            // 
            this.btnDisconnect.BackColor = System.Drawing.SystemColors.GrayText;
            this.btnDisconnect.Location = new System.Drawing.Point(20, 183);
            this.btnDisconnect.Name = "btnDisconnect";
            this.btnDisconnect.Size = new System.Drawing.Size(165, 43);
            this.btnDisconnect.TabIndex = 3;
            this.btnDisconnect.Text = "DISCONNECT";
            this.btnDisconnect.UseVisualStyleBackColor = false;
            this.btnDisconnect.Click += new System.EventHandler(this.btnDisconnect_Click);
            // 
            // comPort
            // 
            this.comPort.FormattingEnabled = true;
            this.comPort.Location = new System.Drawing.Point(31, 33);
            this.comPort.Name = "comPort";
            this.comPort.Size = new System.Drawing.Size(138, 23);
            this.comPort.TabIndex = 0;
            this.comPort.SelectedIndexChanged += new System.EventHandler(this.comPort_SelectedIndexChanged);
            // 
            // comBoudrate
            // 
            this.comBoudrate.FormattingEnabled = true;
            this.comBoudrate.Location = new System.Drawing.Point(31, 75);
            this.comBoudrate.Name = "comBoudrate";
            this.comBoudrate.Size = new System.Drawing.Size(138, 23);
            this.comBoudrate.TabIndex = 1;
            this.comBoudrate.SelectedIndexChanged += new System.EventHandler(this.comBoudrate_SelectedIndexChanged);
            // 
            // btnConnect
            // 
            this.btnConnect.BackColor = System.Drawing.SystemColors.GrayText;
            this.btnConnect.Location = new System.Drawing.Point(20, 125);
            this.btnConnect.Name = "btnConnect";
            this.btnConnect.Size = new System.Drawing.Size(165, 43);
            this.btnConnect.TabIndex = 2;
            this.btnConnect.Text = "CONNECT";
            this.btnConnect.UseVisualStyleBackColor = false;
            this.btnConnect.Click += new System.EventHandler(this.btnConnect_Click);
            // 
            // groupBox2
            // 
            this.groupBox2.BackColor = System.Drawing.SystemColors.ControlDarkDark;
            this.groupBox2.Controls.Add(this.chListbox);
            this.groupBox2.Controls.Add(this.chProgressbar);
            this.groupBox2.Font = new System.Drawing.Font("굴림", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.groupBox2.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.groupBox2.Location = new System.Drawing.Point(483, 289);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(123, 74);
            this.groupBox2.TabIndex = 3;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "Controll Box";
            // 
            // chListbox
            // 
            this.chListbox.AutoSize = true;
            this.chListbox.Font = new System.Drawing.Font("굴림", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.chListbox.Location = new System.Drawing.Point(6, 24);
            this.chListbox.Name = "chListbox";
            this.chListbox.Size = new System.Drawing.Size(73, 16);
            this.chListbox.TabIndex = 0;
            this.chListbox.Text = "ListBox";
            this.chListbox.UseVisualStyleBackColor = true;
            // 
            // chProgressbar
            // 
            this.chProgressbar.AutoSize = true;
            this.chProgressbar.Font = new System.Drawing.Font("굴림", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.chProgressbar.Location = new System.Drawing.Point(6, 52);
            this.chProgressbar.Name = "chProgressbar";
            this.chProgressbar.Size = new System.Drawing.Size(105, 16);
            this.chProgressbar.TabIndex = 3;
            this.chProgressbar.Text = "ProgressBar";
            this.chProgressbar.UseVisualStyleBackColor = true;
            // 
            // chYaw
            // 
            this.chYaw.AutoSize = true;
            this.chYaw.Font = new System.Drawing.Font("굴림", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.chYaw.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.chYaw.Location = new System.Drawing.Point(112, 20);
            this.chYaw.Name = "chYaw";
            this.chYaw.Size = new System.Drawing.Size(86, 16);
            this.chYaw.TabIndex = 3;
            this.chYaw.Text = "Yaw Data";
            this.chYaw.UseVisualStyleBackColor = true;
            // 
            // chPitch
            // 
            this.chPitch.AutoSize = true;
            this.chPitch.Font = new System.Drawing.Font("굴림", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.chPitch.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.chPitch.Location = new System.Drawing.Point(15, 20);
            this.chPitch.Name = "chPitch";
            this.chPitch.Size = new System.Drawing.Size(91, 16);
            this.chPitch.TabIndex = 2;
            this.chPitch.Text = "Pitch Data";
            this.chPitch.UseVisualStyleBackColor = true;
            // 
            // chRoll
            // 
            this.chRoll.AutoSize = true;
            this.chRoll.Font = new System.Drawing.Font("굴림", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.chRoll.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.chRoll.Location = new System.Drawing.Point(15, 51);
            this.chRoll.Name = "chRoll";
            this.chRoll.Size = new System.Drawing.Size(83, 16);
            this.chRoll.TabIndex = 0;
            this.chRoll.Text = "Roll Data";
            this.chRoll.UseVisualStyleBackColor = true;
            // 
            // chPWM
            // 
            this.chPWM.AutoSize = true;
            this.chPWM.Font = new System.Drawing.Font("굴림", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.chPWM.Location = new System.Drawing.Point(15, 51);
            this.chPWM.Name = "chPWM";
            this.chPWM.Size = new System.Drawing.Size(56, 16);
            this.chPWM.TabIndex = 3;
            this.chPWM.Text = "PWM";
            this.chPWM.UseVisualStyleBackColor = true;
            // 
            // chEncoder
            // 
            this.chEncoder.AutoSize = true;
            this.chEncoder.Font = new System.Drawing.Font("굴림", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.chEncoder.Location = new System.Drawing.Point(15, 24);
            this.chEncoder.Name = "chEncoder";
            this.chEncoder.Size = new System.Drawing.Size(112, 16);
            this.chEncoder.TabIndex = 0;
            this.chEncoder.Text = "Encoder Data";
            this.chEncoder.UseVisualStyleBackColor = true;
            // 
            // menuStrip1
            // 
            this.menuStrip1.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.fileToolStripMenuItem,
            this.simulationToolStripMenuItem});
            this.menuStrip1.Location = new System.Drawing.Point(0, 0);
            this.menuStrip1.Name = "menuStrip1";
            this.menuStrip1.Size = new System.Drawing.Size(1337, 24);
            this.menuStrip1.TabIndex = 4;
            this.menuStrip1.Text = "menuStrip1";
            // 
            // fileToolStripMenuItem
            // 
            this.fileToolStripMenuItem.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.saveToolStripMenuItem,
            this.exitToolStripMenuItem,
            this.loadCellToolStripMenuItem});
            this.fileToolStripMenuItem.Name = "fileToolStripMenuItem";
            this.fileToolStripMenuItem.Size = new System.Drawing.Size(37, 20);
            this.fileToolStripMenuItem.Text = "File";
            // 
            // saveToolStripMenuItem
            // 
            this.saveToolStripMenuItem.Name = "saveToolStripMenuItem";
            this.saveToolStripMenuItem.Size = new System.Drawing.Size(125, 22);
            this.saveToolStripMenuItem.Text = "Save";
            this.saveToolStripMenuItem.Click += new System.EventHandler(this.saveToolStripMenuItem_Click);
            // 
            // exitToolStripMenuItem
            // 
            this.exitToolStripMenuItem.Name = "exitToolStripMenuItem";
            this.exitToolStripMenuItem.Size = new System.Drawing.Size(125, 22);
            this.exitToolStripMenuItem.Text = "Exit";
            // 
            // loadCellToolStripMenuItem
            // 
            this.loadCellToolStripMenuItem.Name = "loadCellToolStripMenuItem";
            this.loadCellToolStripMenuItem.Size = new System.Drawing.Size(125, 22);
            this.loadCellToolStripMenuItem.Text = "Load_Cell";
            this.loadCellToolStripMenuItem.Click += new System.EventHandler(this.loadCellToolStripMenuItem_Click_1);
            // 
            // simulationToolStripMenuItem
            // 
            this.simulationToolStripMenuItem.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.startToolStripMenuItem,
            this.endToolStripMenuItem});
            this.simulationToolStripMenuItem.Name = "simulationToolStripMenuItem";
            this.simulationToolStripMenuItem.Size = new System.Drawing.Size(77, 20);
            this.simulationToolStripMenuItem.Text = "Simulation";
            // 
            // startToolStripMenuItem
            // 
            this.startToolStripMenuItem.Name = "startToolStripMenuItem";
            this.startToolStripMenuItem.Size = new System.Drawing.Size(99, 22);
            this.startToolStripMenuItem.Text = "Start";
            // 
            // endToolStripMenuItem
            // 
            this.endToolStripMenuItem.Name = "endToolStripMenuItem";
            this.endToolStripMenuItem.Size = new System.Drawing.Size(99, 22);
            this.endToolStripMenuItem.Text = "End";
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.label7);
            this.groupBox3.Controls.Add(this.label6);
            this.groupBox3.Controls.Add(this.label5);
            this.groupBox3.Controls.Add(this.label4);
            this.groupBox3.Controls.Add(this.label3);
            this.groupBox3.Controls.Add(this.label2);
            this.groupBox3.Controls.Add(this.proYaw);
            this.groupBox3.Controls.Add(this.proRoll);
            this.groupBox3.Controls.Add(this.proPitch);
            this.groupBox3.Controls.Add(this.proThrottle);
            this.groupBox3.Font = new System.Drawing.Font("굴림", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.groupBox3.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.groupBox3.Location = new System.Drawing.Point(695, 41);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Size = new System.Drawing.Size(423, 134);
            this.groupBox3.TabIndex = 5;
            this.groupBox3.TabStop = false;
            this.groupBox3.Text = "ProgressBar";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Font = new System.Drawing.Font("굴림", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.label7.Location = new System.Drawing.Point(16, 106);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(36, 13);
            this.label7.TabIndex = 9;
            this.label7.Text = "Yaw";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Font = new System.Drawing.Font("굴림", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.label6.Location = new System.Drawing.Point(16, 85);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(34, 13);
            this.label6.TabIndex = 8;
            this.label6.Text = "Roll";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Font = new System.Drawing.Font("굴림", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.label5.Location = new System.Drawing.Point(16, 64);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(43, 13);
            this.label5.TabIndex = 7;
            this.label5.Text = "Pitch";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Font = new System.Drawing.Font("굴림", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.label4.Location = new System.Drawing.Point(16, 43);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(61, 13);
            this.label4.TabIndex = 6;
            this.label4.Text = "Throttle";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(354, 25);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(43, 15);
            this.label3.TabIndex = 5;
            this.label3.Text = "1706";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(80, 25);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(34, 15);
            this.label2.TabIndex = 4;
            this.label2.Text = "342";
            // 
            // proYaw
            // 
            this.proYaw.Location = new System.Drawing.Point(83, 106);
            this.proYaw.Name = "proYaw";
            this.proYaw.Size = new System.Drawing.Size(314, 14);
            this.proYaw.TabIndex = 3;
            this.proYaw.Value = 80;
            // 
            // proRoll
            // 
            this.proRoll.Location = new System.Drawing.Point(83, 85);
            this.proRoll.Name = "proRoll";
            this.proRoll.Size = new System.Drawing.Size(314, 15);
            this.proRoll.TabIndex = 2;
            this.proRoll.Value = 30;
            // 
            // proPitch
            // 
            this.proPitch.Location = new System.Drawing.Point(83, 64);
            this.proPitch.Name = "proPitch";
            this.proPitch.Size = new System.Drawing.Size(314, 15);
            this.proPitch.TabIndex = 1;
            this.proPitch.Value = 20;
            // 
            // proThrottle
            // 
            this.proThrottle.Location = new System.Drawing.Point(83, 43);
            this.proThrottle.Name = "proThrottle";
            this.proThrottle.Size = new System.Drawing.Size(314, 15);
            this.proThrottle.TabIndex = 0;
            this.proThrottle.Value = 100;
            // 
            // chart1
            // 
            this.chart1.BorderlineDashStyle = System.Windows.Forms.DataVisualization.Charting.ChartDashStyle.Dash;
            chartArea1.Name = "ChartArea1";
            this.chart1.ChartAreas.Add(chartArea1);
            legend1.Docking = System.Windows.Forms.DataVisualization.Charting.Docking.Top;
            legend1.Name = "Legend1";
            this.chart1.Legends.Add(legend1);
            this.chart1.Location = new System.Drawing.Point(695, 377);
            this.chart1.Name = "chart1";
            series1.ChartArea = "ChartArea1";
            series1.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series1.Legend = "Legend1";
            series1.Name = "Pitch";
            series2.ChartArea = "ChartArea1";
            series2.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series2.Legend = "Legend1";
            series2.Name = "Roll";
            series3.ChartArea = "ChartArea1";
            series3.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series3.Legend = "Legend1";
            series3.Name = "Yaw";
            this.chart1.Series.Add(series1);
            this.chart1.Series.Add(series2);
            this.chart1.Series.Add(series3);
            this.chart1.Size = new System.Drawing.Size(630, 290);
            this.chart1.TabIndex = 8;
            this.chart1.Text = "chart1";
            title1.Font = new System.Drawing.Font("Microsoft YaHei UI", 15.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            title1.Name = "Title1";
            title1.Text = "Gyro Data";
            this.chart1.Titles.Add(title1);
            // 
            // textBox1
            // 
            this.textBox1.Location = new System.Drawing.Point(12, 828);
            this.textBox1.Multiline = true;
            this.textBox1.Name = "textBox1";
            this.textBox1.Size = new System.Drawing.Size(1313, 163);
            this.textBox1.TabIndex = 11;
            // 
            // lblConnection
            // 
            this.lblConnection.AutoSize = true;
            this.lblConnection.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.lblConnection.Location = new System.Drawing.Point(15, 41);
            this.lblConnection.Name = "lblConnection";
            this.lblConnection.Size = new System.Drawing.Size(106, 12);
            this.lblConnection.TabIndex = 12;
            this.lblConnection.Text = "Connection Time ";
            // 
            // groupBox4
            // 
            this.groupBox4.Controls.Add(this.chAcceleration);
            this.groupBox4.Controls.Add(this.chPWM);
            this.groupBox4.Controls.Add(this.chTime);
            this.groupBox4.Controls.Add(this.chEncoder);
            this.groupBox4.Controls.Add(this.groupBox5);
            this.groupBox4.Font = new System.Drawing.Font("굴림", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.groupBox4.ForeColor = System.Drawing.SystemColors.ControlLight;
            this.groupBox4.Location = new System.Drawing.Point(615, 289);
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.Size = new System.Drawing.Size(332, 74);
            this.groupBox4.TabIndex = 15;
            this.groupBox4.TabStop = false;
            this.groupBox4.Text = "MOTOR/RC";
            // 
            // chAcceleration
            // 
            this.chAcceleration.AutoSize = true;
            this.chAcceleration.Font = new System.Drawing.Font("굴림", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.chAcceleration.Location = new System.Drawing.Point(202, 24);
            this.chAcceleration.Name = "chAcceleration";
            this.chAcceleration.Size = new System.Drawing.Size(77, 16);
            this.chAcceleration.TabIndex = 2;
            this.chAcceleration.Text = "Setpoint";
            this.chAcceleration.UseVisualStyleBackColor = true;
            // 
            // chTime
            // 
            this.chTime.AutoSize = true;
            this.chTime.Font = new System.Drawing.Font("굴림", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.chTime.Location = new System.Drawing.Point(133, 24);
            this.chTime.Name = "chTime";
            this.chTime.Size = new System.Drawing.Size(54, 16);
            this.chTime.TabIndex = 1;
            this.chTime.Text = "RPM";
            this.chTime.UseVisualStyleBackColor = true;
            // 
            // groupBox5
            // 
            this.groupBox5.BackColor = System.Drawing.SystemColors.ControlDarkDark;
            this.groupBox5.Font = new System.Drawing.Font("굴림", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.groupBox5.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.groupBox5.Location = new System.Drawing.Point(-253, 0);
            this.groupBox5.Name = "groupBox5";
            this.groupBox5.Size = new System.Drawing.Size(170, 74);
            this.groupBox5.TabIndex = 3;
            this.groupBox5.TabStop = false;
            this.groupBox5.Text = "Check Box";
            // 
            // AUX
            // 
            this.AUX.Controls.Add(this.aux3);
            this.AUX.Controls.Add(this.aux2);
            this.AUX.Controls.Add(this.aux1);
            this.AUX.Font = new System.Drawing.Font("굴림", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.AUX.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.AUX.Location = new System.Drawing.Point(695, 181);
            this.AUX.Name = "AUX";
            this.AUX.Size = new System.Drawing.Size(423, 101);
            this.AUX.TabIndex = 16;
            this.AUX.TabStop = false;
            this.AUX.Text = "AUX";
            // 
            // aux3
            // 
            this.aux3.BackColor = System.Drawing.Color.Sienna;
            this.aux3.Location = new System.Drawing.Point(294, 22);
            this.aux3.Name = "aux3";
            this.aux3.Size = new System.Drawing.Size(113, 64);
            this.aux3.TabIndex = 0;
            this.aux3.Text = "AUX3";
            this.aux3.UseVisualStyleBackColor = false;
            // 
            // aux2
            // 
            this.aux2.BackColor = System.Drawing.Color.Sienna;
            this.aux2.Location = new System.Drawing.Point(160, 22);
            this.aux2.Name = "aux2";
            this.aux2.Size = new System.Drawing.Size(113, 64);
            this.aux2.TabIndex = 0;
            this.aux2.Text = "AUX2";
            this.aux2.UseVisualStyleBackColor = false;
            // 
            // aux1
            // 
            this.aux1.BackColor = System.Drawing.Color.Sienna;
            this.aux1.Cursor = System.Windows.Forms.Cursors.Default;
            this.aux1.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.aux1.Location = new System.Drawing.Point(23, 22);
            this.aux1.Name = "aux1";
            this.aux1.Size = new System.Drawing.Size(113, 64);
            this.aux1.TabIndex = 0;
            this.aux1.Text = "AUX1";
            this.aux1.UseVisualStyleBackColor = false;
            // 
            // chart2
            // 
            chartArea2.Name = "ChartArea1";
            this.chart2.ChartAreas.Add(chartArea2);
            legend2.Docking = System.Windows.Forms.DataVisualization.Charting.Docking.Top;
            legend2.Name = "Legend1";
            this.chart2.Legends.Add(legend2);
            this.chart2.Location = new System.Drawing.Point(12, 377);
            this.chart2.Name = "chart2";
            series4.ChartArea = "ChartArea1";
            series4.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series4.Legend = "Legend1";
            series4.Name = "Encoder";
            series5.ChartArea = "ChartArea1";
            series5.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series5.Legend = "Legend1";
            series5.Name = "Time";
            series6.ChartArea = "ChartArea1";
            series6.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series6.Legend = "Legend1";
            series6.Name = "Acceleration";
            this.chart2.Series.Add(series4);
            this.chart2.Series.Add(series5);
            this.chart2.Series.Add(series6);
            this.chart2.Size = new System.Drawing.Size(677, 290);
            this.chart2.TabIndex = 17;
            this.chart2.Text = "chart2";
            title2.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            title2.Name = "Title1";
            title2.Text = "Motor Data";
            this.chart2.Titles.Add(title2);
            // 
            // chart3
            // 
            chartArea3.Name = "ChartArea1";
            this.chart3.ChartAreas.Add(chartArea3);
            legend3.Name = "PWM";
            this.chart3.Legends.Add(legend3);
            this.chart3.Location = new System.Drawing.Point(12, 674);
            this.chart3.Name = "chart3";
            series7.ChartArea = "ChartArea1";
            series7.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.StepLine;
            series7.Legend = "PWM";
            series7.Name = "PWM";
            series7.YValuesPerPoint = 2;
            this.chart3.Series.Add(series7);
            this.chart3.Size = new System.Drawing.Size(1313, 148);
            this.chart3.TabIndex = 18;
            this.chart3.Text = "chart3";
            // 
            // listBox1
            // 
            this.listBox1.FormattingEnabled = true;
            this.listBox1.ItemHeight = 12;
            this.listBox1.Location = new System.Drawing.Point(12, 57);
            this.listBox1.Name = "listBox1";
            this.listBox1.Size = new System.Drawing.Size(454, 304);
            this.listBox1.TabIndex = 100;
            // 
            // groupBox6
            // 
            this.groupBox6.Controls.Add(this.chYaw);
            this.groupBox6.Controls.Add(this.chRoll);
            this.groupBox6.Controls.Add(this.chPitch);
            this.groupBox6.Font = new System.Drawing.Font("굴림", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.groupBox6.ForeColor = System.Drawing.SystemColors.ControlLight;
            this.groupBox6.Location = new System.Drawing.Point(953, 289);
            this.groupBox6.Name = "groupBox6";
            this.groupBox6.Size = new System.Drawing.Size(275, 73);
            this.groupBox6.TabIndex = 20;
            this.groupBox6.TabStop = false;
            this.groupBox6.Text = "Gyro";
            // 
            // button1
            // 
            this.button1.BackColor = System.Drawing.SystemColors.ControlDarkDark;
            this.button1.Font = new System.Drawing.Font("굴림", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.button1.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.button1.Location = new System.Drawing.Point(1235, 289);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(95, 73);
            this.button1.TabIndex = 21;
            this.button1.Text = "Graph Clear";
            this.button1.UseVisualStyleBackColor = false;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // groupBox7
            // 
            this.groupBox7.Controls.Add(this.label9);
            this.groupBox7.Controls.Add(this.label8);
            this.groupBox7.Controls.Add(this.label1);
            this.groupBox7.Controls.Add(this.btnSender);
            this.groupBox7.Controls.Add(this.comuInt);
            this.groupBox7.Controls.Add(this.txtfloat2);
            this.groupBox7.Controls.Add(this.txtfloat1);
            this.groupBox7.Font = new System.Drawing.Font("굴림", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(129)));
            this.groupBox7.ForeColor = System.Drawing.SystemColors.ControlLightLight;
            this.groupBox7.Location = new System.Drawing.Point(483, 57);
            this.groupBox7.Name = "groupBox7";
            this.groupBox7.Size = new System.Drawing.Size(206, 225);
            this.groupBox7.TabIndex = 101;
            this.groupBox7.TabStop = false;
            this.groupBox7.Text = "Data Tx";
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(3, 103);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(53, 15);
            this.label9.TabIndex = 3;
            this.label9.Text = "Float2";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(3, 69);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(53, 15);
            this.label8.TabIndex = 3;
            this.label8.Text = "Float1";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(9, 32);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(43, 15);
            this.label1.TabIndex = 3;
            this.label1.Text = "uInt8";
            // 
            // btnSender
            // 
            this.btnSender.ForeColor = System.Drawing.SystemColors.ActiveCaptionText;
            this.btnSender.Location = new System.Drawing.Point(19, 146);
            this.btnSender.Name = "btnSender";
            this.btnSender.Size = new System.Drawing.Size(171, 54);
            this.btnSender.TabIndex = 2;
            this.btnSender.Text = "Send";
            this.btnSender.UseVisualStyleBackColor = true;
            this.btnSender.Click += new System.EventHandler(this.btnSender_Click_1);
            // 
            // comuInt
            // 
            this.comuInt.FormattingEnabled = true;
            this.comuInt.Location = new System.Drawing.Point(58, 29);
            this.comuInt.Name = "comuInt";
            this.comuInt.Size = new System.Drawing.Size(132, 23);
            this.comuInt.TabIndex = 1;
            this.comuInt.SelectedIndexChanged += new System.EventHandler(this.comuInt_SelectedIndexChanged);
            // 
            // txtfloat2
            // 
            this.txtfloat2.Location = new System.Drawing.Point(58, 99);
            this.txtfloat2.Name = "txtfloat2";
            this.txtfloat2.Size = new System.Drawing.Size(132, 25);
            this.txtfloat2.TabIndex = 0;
            // 
            // txtfloat1
            // 
            this.txtfloat1.Location = new System.Drawing.Point(58, 64);
            this.txtfloat1.Name = "txtfloat1";
            this.txtfloat1.Size = new System.Drawing.Size(132, 25);
            this.txtfloat1.TabIndex = 0;
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(7F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.SystemColors.ControlDarkDark;
            this.ClientSize = new System.Drawing.Size(1337, 1003);
            this.Controls.Add(this.groupBox7);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.groupBox6);
            this.Controls.Add(this.listBox1);
            this.Controls.Add(this.chart3);
            this.Controls.Add(this.chart2);
            this.Controls.Add(this.AUX);
            this.Controls.Add(this.groupBox4);
            this.Controls.Add(this.lblConnection);
            this.Controls.Add(this.textBox1);
            this.Controls.Add(this.chart1);
            this.Controls.Add(this.groupBox3);
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.menuStrip1);
            this.Name = "Form1";
            this.Text = "3";
            this.groupBox1.ResumeLayout(false);
            this.groupBox2.ResumeLayout(false);
            this.groupBox2.PerformLayout();
            this.menuStrip1.ResumeLayout(false);
            this.menuStrip1.PerformLayout();
            this.groupBox3.ResumeLayout(false);
            this.groupBox3.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.chart1)).EndInit();
            this.groupBox4.ResumeLayout(false);
            this.groupBox4.PerformLayout();
            this.AUX.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.chart2)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.chart3)).EndInit();
            this.groupBox6.ResumeLayout(false);
            this.groupBox6.PerformLayout();
            this.groupBox7.ResumeLayout(false);
            this.groupBox7.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.ComboBox comBoudrate;
        private System.Windows.Forms.ComboBox comPort;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.CheckBox chPitch;
        private System.Windows.Forms.CheckBox chEncoder;
        private System.Windows.Forms.CheckBox chRoll;
        private System.Windows.Forms.Button btnDisconnect;
        private System.Windows.Forms.Button btnConnect;
        private System.Windows.Forms.CheckBox chPWM;
        private System.Windows.Forms.CheckBox chYaw;
        private System.Windows.Forms.MenuStrip menuStrip1;
        private System.Windows.Forms.ToolStripMenuItem fileToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem saveToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem exitToolStripMenuItem;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.ProgressBar proYaw;
        private System.Windows.Forms.ProgressBar proRoll;
        private System.Windows.Forms.ProgressBar proPitch;
        private System.Windows.Forms.ProgressBar proThrottle;
        private System.Windows.Forms.DataVisualization.Charting.Chart chart1;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label2;
        private System.IO.Ports.SerialPort serialPort1;
        private System.Windows.Forms.TextBox textBox1;
        private System.Windows.Forms.Label lblConnection;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.CheckBox chAcceleration;
        private System.Windows.Forms.CheckBox chTime;
        private System.Windows.Forms.GroupBox AUX;
        private System.Windows.Forms.Button aux3;
        private System.Windows.Forms.Button aux2;
        private System.Windows.Forms.Button aux1;
        private System.Windows.Forms.ToolStripMenuItem simulationToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem startToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem endToolStripMenuItem;
        private System.Windows.Forms.DataVisualization.Charting.Chart chart2;
        private System.Windows.Forms.DataVisualization.Charting.Chart chart3;
        private System.Windows.Forms.ListBox listBox1;
        private System.Windows.Forms.CheckBox chListbox;
        private System.Windows.Forms.CheckBox chProgressbar;
        private System.Windows.Forms.GroupBox groupBox5;
        private System.Windows.Forms.GroupBox groupBox6;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.ToolStripMenuItem loadCellToolStripMenuItem;
        private System.Windows.Forms.GroupBox groupBox7;
        private System.Windows.Forms.TextBox txtfloat1;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Button btnSender;
        private System.Windows.Forms.ComboBox comuInt;
        private System.Windows.Forms.TextBox txtfloat2;
    }
}

