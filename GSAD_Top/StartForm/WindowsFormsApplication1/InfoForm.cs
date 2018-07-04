using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace WindowsFormsApplication1
{
    public partial class InfoForm : Form
    {
        public InfoForm(ushort[] date, ushort[] src_num, ushort[] md5, ushort[] id, ushort ver )
        {
            InitializeComponent();

            Date_lbl.Text += ":   " + date[0].ToString() + "." + date[1].ToString() + "." + date[2].ToString() + ".";
            SrcNum_lbl.Text += ":       " +  src_num[0].ToString() + "." + src_num[1].ToString();
            Version_lbl.Text += ":       " + ver.ToString();
            ID_lbl.Text += ":                " + id[0].ToString("X4") + ":" + id[1].ToString("X4") + ":" + id[2].ToString("X4") + ":" + id[3].ToString("X4") + ":" + id[4].ToString("X4") + ":" + id[5].ToString("X4");
            MD5_lbl.Text += ":            " + md5[0].ToString("X4") + ":" + md5[1].ToString("X4") + ":" + md5[2].ToString("X4") + ":" + md5[3].ToString("X4") + ":" + md5[4].ToString("X4") + ":" + md5[5].ToString("X4") + ":" + md5[6].ToString("X4") + ":" + md5[7].ToString("X4");
            //Convert.ToString(id[1], 16)

        }
    }
}
