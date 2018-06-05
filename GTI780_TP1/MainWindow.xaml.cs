using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

// Includes for the Lab
using System.ComponentModel;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using Microsoft.Kinect;
using System.Windows.Forms;
using System.Drawing;

using Emgu.CV;

using Emgu.CV.Structure;
using Emgu.CV.CvEnum;
namespace GTI780_TP1
{
    public partial class MainWindow : Window
    {
        // Number of bytes per pixel for the format used in this project
        private int BYTESPERPIXELS = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;

        // Size of the target display screen
        private double screenWidth;
        private double screenHeight;

        // Bitmaps to display
        private WriteableBitmap colorBitmap = null;
        private WriteableBitmap depthBitmap = null;
        //private WriteableBitmap headerBitmap = null;

        // Kinect Sensor
        private KinectSensor kinectSensor = null;

        // Frame Reader
        private MultiSourceFrameReader multiSourceFrameReader = null;

        // CoordinateMapper
        private CoordinateMapper coordinateMapper = null;

        // Mapping de la couleur vers la profondeur
        private DepthSpacePoint[] mapColorToDepth = null;

        private int bitmapBackBufferSize = 0;

        private int depthToBytes = 8000 / 256;

        private int depthPixelWidth = 0;
        private int depthPixelHeight = 0;




        public MainWindow()
        {
            //Initialize UI
            InitializeComponent();

            //Sets the correct size to the display components
            InitializeComponentsSize();

            //Connect to sensor
            this.kinectSensor = KinectSensor.GetDefault();

            //CoordinateMapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            //Open reader for multiframes
            this.multiSourceFrameReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.Color);

            //For the color to depth mapping
            int colorWidth = this.kinectSensor.ColorFrameSource.FrameDescription.Width;
            int colorHeight = this.kinectSensor.ColorFrameSource.FrameDescription.Height;
            this.mapColorToDepth = new DepthSpacePoint[colorWidth * colorHeight];

            //Instanciate the WriteableBitmaps used to display the kinect frames
            this.colorBitmap = new WriteableBitmap(colorWidth, colorHeight, 96.0, 96.0, PixelFormats.Bgra32, null);
            this.depthBitmap = new WriteableBitmap(colorWidth, colorHeight, 96.0, 96.0, PixelFormats.Gray8, null);

            // Calculate the WriteableBitmap back buffer size
            this.bitmapBackBufferSize = ((this.colorBitmap.BackBufferStride * (this.colorBitmap.PixelHeight - 1)) + (this.colorBitmap.PixelWidth * this.BYTESPERPIXELS));

            //Handle the arrival of a frame
            this.multiSourceFrameReader.MultiSourceFrameArrived += this.Reader_FrameArrived;

            //Open the kinect sensor
            this.kinectSensor.Open();

            //Sets the context for the data binding
            this.DataContext = this;

        }




        // This event will be called whenever the multi Frame Reader receives a new frame
        void Reader_FrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            //Store the depth and the color frame 
            ColorFrame colorFrame = null;
            DepthFrame depthFrame = null;

            //Store the state of the frame lock
            bool isLocked = false;

            //Acquire a new frame
            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();

            //If the Frame has expired or is invalid, return
            if (multiSourceFrame == null) { return; }

            try
            {

                depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame();

                // If the frames have expired or are invalid, return.
                if (depthFrame == null) { return; }

                /**
                 * 
                 * ********Depth*********
                 * 
                 **/

                depthPixelHeight = this.depthBitmap.PixelHeight;
                depthPixelWidth = this.depthBitmap.PixelWidth;

                int depthHeight = depthFrame.FrameDescription.Height;
                int depthWidth = depthFrame.FrameDescription.Width;
                ushort depthMinDistance = depthFrame.DepthMinReliableDistance;

                ushort[] frameArrayData = new ushort[depthFrame.FrameDescription.Width * depthFrame.FrameDescription.Height];
                depthFrame.CopyFrameDataToArray(frameArrayData);

                //Map the frame from color space to depth space using data from the buffer
                //Using kinectbuffer to get underlying buffer data
                using (KinectBuffer data = depthFrame.LockImageBuffer())
                {
                    this.coordinateMapper.MapColorFrameToDepthSpaceUsingIntPtr(data.UnderlyingBuffer,data.Size,this.mapColorToDepth);
                }

                // We are done with the depthFrame, dispose of it
                depthFrame.Dispose();
                depthFrame = null;


                /**
                 * 
                 * ********Color******
                 * 
                 **/

                colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame();

                // If the frames have expired or are invalid, return.
                if (colorFrame == null) { return; }

                int colorHeight = colorFrame.FrameDescription.Height;
                int colorWidth = colorFrame.FrameDescription.Width;

                // Lock the colorBitmap while we write in it.
                this.colorBitmap.Lock();
                isLocked = true;

                // Check for correct size
                if (colorWidth == this.colorBitmap.Width && colorHeight == this.colorBitmap.Height)
                {
                    //write the new color frame data to the display bitmap
                    colorFrame.CopyConvertedFrameDataToIntPtr(this.colorBitmap.BackBuffer, (uint)this.bitmapBackBufferSize, ColorImageFormat.Bgra);
                }

                // We are done with the colorFrame, dispose of it
                colorFrame.Dispose();
                colorFrame = null;



                /**
                 * To check this part.
                 */


                //new 1920 x 1080 depth array
                byte[] depthPixelArray = new byte[(int)this.colorBitmap.Width * (int)this.colorBitmap.Height];


                //Les pointeurs et les mémoires tampons de taille fixe ne peuvent être utilisés que dans un contexte unsafe
                unsafe
                {
                    fixed (DepthSpacePoint* colorMappedToDepthPointsPointer = this.mapColorToDepth)
                    {
                        // Treat the color data as 4-byte pixels
                        uint* bitmapPixelsPointer = (uint*)this.colorBitmap.BackBuffer;

                        // Loop over each row and column of the color image
                        // Zero out any pixels that don't correspond to a body index
                        for (int colorIndex = 0; colorIndex < this.mapColorToDepth.Length; ++colorIndex)
                        {
                            float colorMappedToDepthX = colorMappedToDepthPointsPointer[colorIndex].X;
                            float colorMappedToDepthY = colorMappedToDepthPointsPointer[colorIndex].Y;

                            // The sentinel value is -inf, -inf, meaning that no depth pixel corresponds to this color pixel.
                            if (!float.IsNegativeInfinity(colorMappedToDepthX) &&
                                !float.IsNegativeInfinity(colorMappedToDepthY))
                            {
                                // Make sure the depth pixel maps to valid point in color space
                                int depthX = (int)(colorMappedToDepthX + 0.5f);
                                int depthY = (int)(colorMappedToDepthY + 0.5f);

                                // If the point isnt valid, no body index.
                                if ((depthX >= 0) && (depthX < depthWidth) && (depthY >= 0) && (depthY < depthHeight))
                                {
                                    int depthIndex = (depthY * depthWidth) + depthX;
                                    ushort depth = frameArrayData[depthIndex];
                                    depthPixelArray[colorIndex] = (byte)(depth >= depthMinDistance && depth <= ushort.MaxValue ? (depth / depthToBytes) : 0);
                                }
                            }
                        }
                        //this.AddHeader(this.colorBitmap);

                        this.colorBitmap.AddDirtyRect(new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight));

                        this.depthBitmap.WritePixels(new Int32Rect(0, 0, depthPixelWidth, depthPixelHeight), depthPixelArray, depthPixelWidth,0);

                    }
                }
            }
            finally
            {
                if (isLocked)
                {
                    this.colorBitmap.Unlock();
                }
                if (depthFrame != null)
                {
                    depthFrame.Dispose();
                }
                if (colorFrame != null)
                {
                    colorFrame.Dispose();
                }
            }
        } //END Reader_FrameArrived()

        public ImageSource ImageSource1
        {
            get
            {
                return this.colorBitmap;
            }
        }

        public ImageSource ImageSource2
        {
            get
            {
                return this.depthBitmap;
            }
        }

        private void InitializeComponentsSize()
        {
            // Get the screen size
            Screen[] screens = Screen.AllScreens;
            this.screenWidth = screens[0].Bounds.Width;
            this.screenHeight = screens[0].Bounds.Height;

            // Make the application full screen
            this.Width = this.screenWidth;
            this.Height = this.screenHeight;
            this.MainWindow1.Width = this.screenWidth;
            this.MainWindow1.Height = this.screenHeight;

            // Make the Grid container full screen
            this.Grid1.Width = this.screenWidth;
            this.Grid1.Height = this.screenHeight;

            // Make the PictureBox1 half the screen width and full screen height
            this.PictureBox1.Width = this.screenWidth / 2;
            this.PictureBox1.Height = this.screenHeight;

            // Make the PictureBox2 half the screen width and full screen height
            this.PictureBox2.Width = this.screenWidth / 2;
            this.PictureBox2.Margin = new Thickness(0, 0, 0, 0);
            this.PictureBox2.Height = this.screenHeight;
        }

        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.multiSourceFrameReader != null)
            {
                this.multiSourceFrameReader.Dispose();
                this.multiSourceFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }
    }
}