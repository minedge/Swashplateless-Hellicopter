
namespace Project_Main
{
    partial class LoadCell_Form
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            System.Windows.Forms.DataVisualization.Charting.ChartArea chartArea1 = new System.Windows.Forms.DataVisualization.Charting.ChartArea();
            System.Windows.Forms.DataVisualization.Charting.Legend legend1 = new System.Windows.Forms.DataVisualization.Charting.Legend();
            System.Windows.Forms.DataVisualization.Charting.Series series1 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Series series2 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Series series3 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Series series4 = new System.Windows.Forms.DataVisualization.Charting.Series();
            this.serialPort2 = new System.IO.Ports.SerialPort(this.components);
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.listBox1 = new System.Windows.Forms.ListBox();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.comboud = new System.Windows.Forms.ComboBox();
            this.comport = new System.Windows.Forms.ComboBox();
            this.btconnect = new System.Windows.Forms.Button();
            this.btdisconnect = new System.Windows.Forms.Button();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.chgraph = new System.Windows.Forms.CheckBox();
            this.btrear = new System.Windows.Forms.Button();
            this.btright = new System.Windows.Forms.Button();
            this.btfront = new System.Windows.Forms.Button();
            this.btleft = new System.Windows.Forms.Button();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.chart1 = new System.Windows.Forms.DataVisualization.Charting.Chart();
            this.button = new System.Windows.Forms.CheckBox();
            this.listbox = new System.Windows.Forms.CheckBox();
            this.groupBox1.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.groupBox3.SuspendLayout();
            this.groupBox4.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.chart1)).BeginInit();
            this.SuspendLayout();
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.listBox1);
            this.groupBox1.Location = new System.Drawing.Point(12, 13);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(479, 541);
            this.groupBox1.TabIndex = 0;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Data";
            // 
            // listBox1
            // 
            this.listBox1.FormattingEnabled = true;
            this.listBox1.ItemHeight = 12;
            this.listBox1.Location = new System.Drawing.Point(7, 21);
            this.listBox1.Name = "listBox1";
            this.listBox1.Size = new System.Drawing.Size(466, 508);
            this.listBox1.TabIndex = 0;
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.comboud);
            this.groupBox2.Controls.Add(this.comport);
            this.groupBox2.Controls.Add(this.btconnect);
            this.groupBox2.Controls.Add(this.btdisconnect);
            this.groupBox2.Location = new System.Drawing.Point(994, 13);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(183, 174);
            this.groupBox2.TabIndex = 1;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "Connection";
            // 
            // comboud
            // 
            this.comboud.FormattingEnabled = true;
            this.comboud.Location = new System.Drawing.Point(29, 62);
            this.comboud.Name = "comboud";
            this.comboud.Size = new System.Drawing.Size(132, 20);
            this.comboud.TabIndex = 1;
            this.comboud.SelectedIndexChanged += new System.EventHandler(this.comboud_SelectedIndexChanged);
            // 
            // comport
            // 
            this.comport.FormattingEnabled = true;
            this.comport.Location = new System.Drawing.Point(29, 28);
            this.comport.Name = "comport";
            this.comport.Size = new System.Drawing.Size(132, 20);
            this.comport.TabIndex = 0;
            this.comport.SelectedIndexChanged += new System.EventHandler(this.comport_SelectedIndexChanged);
            // 
            // btconnect
            // 
            this.btconnect.Location = new System.Drawing.Point(29, 103);
            this.btconnect.Name = "btconnect";
            this.btconnect.Size = new System.Drawing.Size(132, 28);
            this.btconnect.TabIndex = 0;
            this.btconnect.Text = "Connect";
            this.btconnect.UseVisualStyleBackColor = true;
            this.btconnect.Click += new System.EventHandler(this.btconnect_Click);
            // 
            // btdisconnect
            // 
            this.btdisconnect.Location = new System.Drawing.Point(29, 137);
            this.btdisconnect.Name = "btdisconnect";
            this.btdisconnect.Size = new System.Drawing.Size(132, 28);
            this.btdisconnect.TabIndex = 0;
            this.btdisconnect.Text = "Disconnect";
            this.btdisconnect.UseVisualStyleBackColor = true;
            this.btdisconnect.Click += new System.EventHandler(this.btdisconnect_Click);
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.btrear);
            this.groupBox3.Controls.Add(this.btright);
            this.groupBox3.Controls.Add(this.btfront);
            this.groupBox3.Controls.Add(this.btleft);
            this.groupBox3.Location = new System.Drawing.Point(497, 13);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Size = new System.Drawing.Size(491, 174);
            this.groupBox3.TabIndex = 2;
            this.groupBox3.TabStop = false;
            this.groupBox3.Text = "Load_Cell";
            // 
            // chgraph
            // 
            this.chgraph.AutoSize = true;
            this.chgraph.Location = new System.Drawing.Point(585, 128);
            this.chgraph.Name = "chgraph";
            this.chgraph.Size = new System.Drawing.Size(58, 16);
            this.chgraph.TabIndex = 1;
            this.chgraph.Text = "Graph";
            this.chgraph.UseVisualStyleBackColor = true;
            // 
            // btrear
            // 
            this.btrear.Location = new System.Drawing.Point(192, 111);
            this.btrear.Name = "btrear";
            this.btrear.Size = new System.Drawing.Size(116, 54);
            this.btrear.TabIndex = 0;
            this.btrear.Text = "Rear";
            this.btrear.UseVisualStyleBackColor = true;
            // 
            // btright
            // 
            this.btright.Location = new System.Drawing.Point(336, 62);
            this.btright.Name = "btright";
            this.btright.Size = new System.Drawing.Size(116, 54);
            this.btright.TabIndex = 0;
            this.btright.Text = "Right";
            this.btright.UseVisualStyleBackColor = true;
            // 
            // btfront
            // 
            this.btfront.Location = new System.Drawing.Point(192, 10);
            this.btfront.Name = "btfront";
            this.btfront.Size = new System.Drawing.Size(116, 54);
            this.btfront.TabIndex = 0;
            this.btfront.Text = "Front";
            this.btfront.UseVisualStyleBackColor = true;
            // 
            // btleft
            // 
            this.btleft.Location = new System.Drawing.Point(48, 62);
            this.btleft.Name = "btleft";
            this.btleft.Size = new System.Drawing.Size(116, 54);
            this.btleft.TabIndex = 0;
            this.btleft.Text = "Left";
            this.btleft.UseVisualStyleBackColor = true;
            // 
            // groupBox4
            // 
            this.groupBox4.Controls.Add(this.listbox);
            this.groupBox4.Controls.Add(this.button);
            this.groupBox4.Controls.Add(this.chgraph);
            this.groupBox4.Controls.Add(this.chart1);
            this.groupBox4.Location = new System.Drawing.Point(497, 193);
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.Size = new System.Drawing.Size(680, 361);
            this.groupBox4.TabIndex = 2;
            this.groupBox4.TabStop = false;
            this.groupBox4.Text = "Graph";
            // 
            // chart1
            // 
            chartArea1.Name = "ChartArea1";
            this.chart1.ChartAreas.Add(chartArea1);
            legend1.Name = "Legend1";
            this.chart1.Legends.Add(legend1);
            this.chart1.Location = new System.Drawing.Point(7, 21);
            this.chart1.Name = "chart1";
            series1.ChartArea = "ChartArea1";
            series1.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series1.Legend = "Legend1";
            series1.Name = "Front";
            series2.ChartArea = "ChartArea1";
            series2.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series2.Legend = "Legend1";
            series2.Name = "Rear";
            series3.ChartArea = "ChartArea1";
            series3.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series3.Legend = "Legend1";
            series3.Name = "Left";
            series4.ChartArea = "ChartArea1";
            series4.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series4.Legend = "Legend1";
            series4.Name = "Right";
            this.chart1.Series.Add(series1);
            this.chart1.Series.Add(series2);
            this.chart1.Series.Add(series3);
            this.chart1.Series.Add(series4);
            this.chart1.Size = new System.Drawing.Size(667, 334);
            this.chart1.TabIndex = 0;
            this.chart1.Text = "chart1";
            // 
            // button
            // 
            this.button.AutoSize = true;
            this.button.Location = new System.Drawing.Point(585, 150);
            this.button.Name = "button";
            this.button.Size = new System.Drawing.Size(80, 16);
            this.button.TabIndex = 1;
            this.button.Text = "Load_Cell";
            this.button.UseVisualStyleBackColor = true;
            // 
            // listbox
            // 
            this.listbox.AutoSize = true;
            this.listbox.Location = new System.Drawing.Point(585, 172);
            this.listbox.Name = "listbox";
            this.listbox.Size = new System.Drawing.Size(66, 16);
            this.listbox.TabIndex = 1;
            this.listbox.Text = "ListBox";
            this.listbox.UseVisualStyleBackColor = true;
            // 
            // LoadCell_Form
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(7F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1187, 561);
            this.Controls.Add(this.groupBox4);
            this.Controls.Add(this.groupBox3);
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.groupBox1);
            this.Name = "LoadCell_Form";
            this.Text = "LoadCell_Form";
            this.groupBox1.ResumeLayout(false);
            this.groupBox2.ResumeLayout(false);
            this.groupBox3.ResumeLayout(false);
            this.groupBox4.ResumeLayout(false);
            this.groupBox4.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.chart1)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private System.IO.Ports.SerialPort serialPort2;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.ListBox listBox1;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.Button btrear;
        private System.Windows.Forms.Button btright;
        private System.Windows.Forms.Button btfront;
        private System.Windows.Forms.Button btleft;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.DataVisualization.Charting.Chart chart1;
        private System.Windows.Forms.ComboBox comboud;
        private System.Windows.Forms.ComboBox comport;
        private System.Windows.Forms.Button btconnect;
        private System.Windows.Forms.Button btdisconnect;
        private System.Windows.Forms.CheckBox chgraph;
        private System.Windows.Forms.CheckBox listbox;
        private System.Windows.Forms.CheckBox button;
    }
}