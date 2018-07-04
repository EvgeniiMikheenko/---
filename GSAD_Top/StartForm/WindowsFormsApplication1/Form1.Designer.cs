namespace WindowsFormsApplication1
{
    partial class Form1
    {
        /// <summary>
        /// Обязательная переменная конструктора.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Освободить все используемые ресурсы.
        /// </summary>
        /// <param name="disposing">истинно, если управляемый ресурс должен быть удален; иначе ложно.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Код, автоматически созданный конструктором форм Windows

        /// <summary>
        /// Требуемый метод для поддержки конструктора — не изменяйте 
        /// содержимое этого метода с помощью редактора кода.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Form1));
            this.menuStrip1 = new System.Windows.Forms.MenuStrip();
            this.ConnectMenu = new System.Windows.Forms.ToolStripMenuItem();
            this.отключитьToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.настройкаToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.toolStripMenuItem1 = new System.Windows.Forms.ToolStripMenuItem();
            this.COM = new System.IO.Ports.SerialPort(this.components);
            this.panel1 = new System.Windows.Forms.Panel();
            this.Status_lbl = new System.Windows.Forms.Label();
            this.Rdy_panel = new System.Windows.Forms.Panel();
            this.Rdy_lbl = new System.Windows.Forms.Label();
            this.Alarm_panel = new System.Windows.Forms.Panel();
            this.Alarm_lbl = new System.Windows.Forms.Label();
            this.CHB_btn = new System.Windows.Forms.CheckBox();
            this.menuStrip1.SuspendLayout();
            this.panel1.SuspendLayout();
            this.Rdy_panel.SuspendLayout();
            this.Alarm_panel.SuspendLayout();
            this.SuspendLayout();
            // 
            // menuStrip1
            // 
            this.menuStrip1.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.ConnectMenu,
            this.toolStripMenuItem1});
            this.menuStrip1.Location = new System.Drawing.Point(0, 0);
            this.menuStrip1.Name = "menuStrip1";
            this.menuStrip1.Size = new System.Drawing.Size(452, 24);
            this.menuStrip1.TabIndex = 1;
            this.menuStrip1.Text = "menuStrip1";
            // 
            // ConnectMenu
            // 
            this.ConnectMenu.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.отключитьToolStripMenuItem,
            this.настройкаToolStripMenuItem});
            this.ConnectMenu.Name = "ConnectMenu";
            this.ConnectMenu.Size = new System.Drawing.Size(97, 20);
            this.ConnectMenu.Text = "Подключение";
            this.ConnectMenu.DropDownItemClicked += new System.Windows.Forms.ToolStripItemClickedEventHandler(this.ConnectMenu_DropDownItemClicked);
            this.ConnectMenu.Click += new System.EventHandler(this.ConnectMenu_Click);
            // 
            // отключитьToolStripMenuItem
            // 
            this.отключитьToolStripMenuItem.Name = "отключитьToolStripMenuItem";
            this.отключитьToolStripMenuItem.Size = new System.Drawing.Size(152, 22);
            this.отключитьToolStripMenuItem.Text = "Отключить";
            // 
            // настройкаToolStripMenuItem
            // 
            this.настройкаToolStripMenuItem.Name = "настройкаToolStripMenuItem";
            this.настройкаToolStripMenuItem.Size = new System.Drawing.Size(152, 22);
            this.настройкаToolStripMenuItem.Text = "Настройка";
            // 
            // toolStripMenuItem1
            // 
            this.toolStripMenuItem1.Name = "toolStripMenuItem1";
            this.toolStripMenuItem1.Size = new System.Drawing.Size(93, 20);
            this.toolStripMenuItem1.Text = "Информация";
            this.toolStripMenuItem1.Click += new System.EventHandler(this.toolStripMenuItem1_Click);
            // 
            // COM
            // 
            this.COM.BaudRate = 115200;
            // 
            // panel1
            // 
            this.panel1.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.panel1.BackColor = System.Drawing.SystemColors.ButtonFace;
            this.panel1.Controls.Add(this.Status_lbl);
            this.panel1.Location = new System.Drawing.Point(0, 467);
            this.panel1.Name = "panel1";
            this.panel1.Size = new System.Drawing.Size(171, 27);
            this.panel1.TabIndex = 2;
            // 
            // Status_lbl
            // 
            this.Status_lbl.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.Status_lbl.AutoSize = true;
            this.Status_lbl.Location = new System.Drawing.Point(3, 7);
            this.Status_lbl.Name = "Status_lbl";
            this.Status_lbl.Size = new System.Drawing.Size(61, 13);
            this.Status_lbl.TabIndex = 0;
            this.Status_lbl.Text = "Состояние";
            // 
            // Rdy_panel
            // 
            this.Rdy_panel.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.Rdy_panel.BackColor = System.Drawing.SystemColors.ActiveBorder;
            this.Rdy_panel.Controls.Add(this.Rdy_lbl);
            this.Rdy_panel.Location = new System.Drawing.Point(0, 434);
            this.Rdy_panel.Name = "Rdy_panel";
            this.Rdy_panel.Size = new System.Drawing.Size(452, 27);
            this.Rdy_panel.TabIndex = 4;
            // 
            // Rdy_lbl
            // 
            this.Rdy_lbl.AutoSize = true;
            this.Rdy_lbl.Font = new System.Drawing.Font("Microsoft JhengHei UI", 15.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.Rdy_lbl.Location = new System.Drawing.Point(185, 1);
            this.Rdy_lbl.Name = "Rdy_lbl";
            this.Rdy_lbl.Size = new System.Drawing.Size(104, 26);
            this.Rdy_lbl.TabIndex = 0;
            this.Rdy_lbl.Text = "Не готов";
            this.Rdy_lbl.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // Alarm_panel
            // 
            this.Alarm_panel.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.Alarm_panel.BackColor = System.Drawing.SystemColors.ButtonHighlight;
            this.Alarm_panel.Controls.Add(this.Alarm_lbl);
            this.Alarm_panel.Location = new System.Drawing.Point(0, 28);
            this.Alarm_panel.Name = "Alarm_panel";
            this.Alarm_panel.Size = new System.Drawing.Size(452, 400);
            this.Alarm_panel.TabIndex = 5;
            // 
            // Alarm_lbl
            // 
            this.Alarm_lbl.AutoSize = true;
            this.Alarm_lbl.Font = new System.Drawing.Font("Microsoft Sans Serif", 48F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(204)));
            this.Alarm_lbl.Location = new System.Drawing.Point(-7, 157);
            this.Alarm_lbl.MinimumSize = new System.Drawing.Size(100, 100);
            this.Alarm_lbl.Name = "Alarm_lbl";
            this.Alarm_lbl.Size = new System.Drawing.Size(475, 100);
            this.Alarm_lbl.TabIndex = 0;
            this.Alarm_lbl.Text = "Не подключен";
            this.Alarm_lbl.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // CHB_btn
            // 
            this.CHB_btn.Appearance = System.Windows.Forms.Appearance.Button;
            this.CHB_btn.AutoSize = true;
            this.CHB_btn.Location = new System.Drawing.Point(199, 1);
            this.CHB_btn.Name = "CHB_btn";
            this.CHB_btn.Size = new System.Drawing.Size(138, 23);
            this.CHB_btn.TabIndex = 6;
            this.CHB_btn.Text = "Поверх остальных окон";
            this.CHB_btn.UseVisualStyleBackColor = true;
            this.CHB_btn.CheckedChanged += new System.EventHandler(this.CHB_btn_CheckedChanged);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(452, 496);
            this.Controls.Add(this.CHB_btn);
            this.Controls.Add(this.Alarm_panel);
            this.Controls.Add(this.Rdy_panel);
            this.Controls.Add(this.panel1);
            this.Controls.Add(this.menuStrip1);
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.MaximizeBox = false;
            this.MinimumSize = new System.Drawing.Size(100, 200);
            this.Name = "Form1";
            this.Text = "GSA-D";
            this.menuStrip1.ResumeLayout(false);
            this.menuStrip1.PerformLayout();
            this.panel1.ResumeLayout(false);
            this.panel1.PerformLayout();
            this.Rdy_panel.ResumeLayout(false);
            this.Rdy_panel.PerformLayout();
            this.Alarm_panel.ResumeLayout(false);
            this.Alarm_panel.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.MenuStrip menuStrip1;
        private System.Windows.Forms.ToolStripMenuItem ConnectMenu;
        private System.Windows.Forms.ToolStripMenuItem отключитьToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem настройкаToolStripMenuItem;
        private System.IO.Ports.SerialPort COM;
        private System.Windows.Forms.Panel panel1;
        private System.Windows.Forms.Label Status_lbl;
        private System.Windows.Forms.ToolStripMenuItem toolStripMenuItem1;
        private System.Windows.Forms.Panel Rdy_panel;
        private System.Windows.Forms.Label Rdy_lbl;
        private System.Windows.Forms.Panel Alarm_panel;
        private System.Windows.Forms.Label Alarm_lbl;
        private System.Windows.Forms.CheckBox CHB_btn;
    }
}

