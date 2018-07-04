namespace WindowsFormsApplication1
{
    partial class InfoForm
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
            this.Date_lbl = new System.Windows.Forms.Label();
            this.SrcNum_lbl = new System.Windows.Forms.Label();
            this.MD5_lbl = new System.Windows.Forms.Label();
            this.ID_lbl = new System.Windows.Forms.Label();
            this.Version_lbl = new System.Windows.Forms.Label();
            this.SuspendLayout();
            // 
            // Date_lbl
            // 
            this.Date_lbl.AutoSize = true;
            this.Date_lbl.Location = new System.Drawing.Point(12, 27);
            this.Date_lbl.Name = "Date_lbl";
            this.Date_lbl.Size = new System.Drawing.Size(107, 13);
            this.Date_lbl.TabIndex = 0;
            this.Date_lbl.Text = "Дата производства";
            // 
            // SrcNum_lbl
            // 
            this.SrcNum_lbl.AutoSize = true;
            this.SrcNum_lbl.Location = new System.Drawing.Point(12, 54);
            this.SrcNum_lbl.Name = "SrcNum_lbl";
            this.SrcNum_lbl.Size = new System.Drawing.Size(96, 13);
            this.SrcNum_lbl.TabIndex = 1;
            this.SrcNum_lbl.Text = "Номер источника";
            // 
            // MD5_lbl
            // 
            this.MD5_lbl.AutoSize = true;
            this.MD5_lbl.Location = new System.Drawing.Point(12, 82);
            this.MD5_lbl.Name = "MD5_lbl";
            this.MD5_lbl.Size = new System.Drawing.Size(30, 13);
            this.MD5_lbl.TabIndex = 2;
            this.MD5_lbl.Text = "MD5";
            // 
            // ID_lbl
            // 
            this.ID_lbl.AutoSize = true;
            this.ID_lbl.Location = new System.Drawing.Point(13, 108);
            this.ID_lbl.Name = "ID_lbl";
            this.ID_lbl.Size = new System.Drawing.Size(18, 13);
            this.ID_lbl.TabIndex = 3;
            this.ID_lbl.Text = "ID";
            // 
            // Version_lbl
            // 
            this.Version_lbl.AutoSize = true;
            this.Version_lbl.Location = new System.Drawing.Point(13, 131);
            this.Version_lbl.Name = "Version_lbl";
            this.Version_lbl.Size = new System.Drawing.Size(44, 13);
            this.Version_lbl.TabIndex = 4;
            this.Version_lbl.Text = "Версия";
            // 
            // InfoForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(314, 153);
            this.Controls.Add(this.Version_lbl);
            this.Controls.Add(this.ID_lbl);
            this.Controls.Add(this.MD5_lbl);
            this.Controls.Add(this.SrcNum_lbl);
            this.Controls.Add(this.Date_lbl);
            this.MaximizeBox = false;
            this.MaximumSize = new System.Drawing.Size(330, 191);
            this.MinimizeBox = false;
            this.MinimumSize = new System.Drawing.Size(330, 191);
            this.Name = "InfoForm";
            this.ShowIcon = false;
            this.Text = "InfoForm";
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Label Date_lbl;
        private System.Windows.Forms.Label SrcNum_lbl;
        private System.Windows.Forms.Label MD5_lbl;
        private System.Windows.Forms.Label ID_lbl;
        private System.Windows.Forms.Label Version_lbl;
    }
}