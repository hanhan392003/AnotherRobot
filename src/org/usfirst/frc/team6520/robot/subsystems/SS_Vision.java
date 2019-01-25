package org.usfirst.frc.team6520.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team6520.robot.RobotMap;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class SS_Vision extends Subsystem implements PIDSource {
	Thread m_visionThread;
	double centerX = 0, centerY = 0;
	// các biến chỉ tọa độ X, Y trọng tâm của vật thể được detect
	Preferences prefs = Preferences.getInstance();
	// tạo file preference mới cho smartdashboard

	public void vision() {
		m_visionThread = new Thread(() -> {
		// khởi tạo camera và đặt resolution
		// resolution khá là quan trọng vì hệ thống điều khiển sân đấu có
		// giới hạn về bandwidth nên cần để dữ liệu thấp
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(320, 240);
		camera.setResolution(320, 240);
		camera.setExposureManual(35);

		// khởi tạo CvSink là một vật thể host video từ cam
		CvSink cvSink = CameraServer.getInstance().getVideo();
		// khởi tạo vật thể để output processed feed đến smartdashboard
		CvSource outputStream = CameraServer.getInstance().putVideo("Vision", 320, 240);

		// mat = ma trận
		// tạo mat để thể hiện dữ liệu ảnh
		Mat mat = new Mat();
		// tạo mat thể hiện kernel sử dụng để xử lý (sẽ nói sau).
		// MORPH_RECT tức là hình dạng của kernel này là hình chữ nhật, kích cỡ 3x3
		Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));

		// tạo danh sách để lưu lại những object detect được
		List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		List<RotatedRect> boundRect = new ArrayList<RotatedRect>();
		
		// một mat nào đó cần thiết cho thuật tìm contour
		Mat hierarchy = new Mat();

		// load các giá trị từ file preference, có default backup nếu pref chưa tồn tại
		int R = prefs.getInt("R", 0); // màu đỏ
		int G = prefs.getInt("G", 0); // màu lục
		int B = prefs.getInt("B", 0); // màu xanh lam
		int RE = prefs.getInt("RE", 20); // khoảng của đỏ
		int GE = prefs.getInt("GE", 20); // khoảng của lục
		int BE = prefs.getInt("BE", 20); // khoảng của xanh

		// đưa các giá trị đó lên smartdashboard
		SmartDashboard.putNumber("R", R);
		SmartDashboard.putNumber("G", G);
		SmartDashboard.putNumber("B", B);
		SmartDashboard.putNumber("RE", RE);
		SmartDashboard.putNumber("GE", GE);
		SmartDashboard.putNumber("BE", BE);

		while (!Thread.interrupted()) {
//		while (true) {
			contours.removeAll(contours);
			boundRect.removeAll(boundRect);
			// lưu các giá trị vào preference để khi bot reset hoặc disable/reenable
			// thì không mất giá trị
			prefs.putInt("R", R);
			prefs.putInt("G", G);
			prefs.putInt("B", B);
			prefs.putInt("RE", RE);
			prefs.putInt("GE", GE);
			prefs.putInt("BE", BE);

			if (cvSink.grabFrame(mat) == 0) {
				// nếu cvSink không vớ được ảnh thì skip loop
				// failsafe measure
				outputStream.notifyError(cvSink.getError());
				continue;
			}

			// lấy giá trị từ dashboard đưa vào robot
			R = (int) SmartDashboard.getNumber("R", 255);
			G = (int) SmartDashboard.getNumber("G", 255);
			B = (int) SmartDashboard.getNumber("B", 255);
			RE = (int) SmartDashboard.getNumber("RE", 20);
			GE = (int) SmartDashboard.getNumber("GE", 20);
			BE = (int) SmartDashboard.getNumber("BE", 20);

			// convert ảnh trong mat từ định dạng RGB ra HSV
			Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);

			// thuật toán để loại các pixel không nằm trong khoảng cho trước
			// và giữ lại những pixel nằm trong khoảng cho trước
			// khoảng của mỗi màu là (màu - lỗi màu < pixel < màu + lỗi màu)
			Core.inRange(mat, new Scalar(R - RE, G - GE, B - BE), new Scalar(R + RE, G + GE, B + BE), mat);

			// sử dụng kernel để loại bỏ các lỗ hổng trong các cụm pixel còn lại
			Imgproc.dilate(mat, mat, kernel);

			// thuật toán tìm viền ngoài của các cụm pixel
			// từ đó group các cụm pixel lại thành vật thể
			// danh sách contour được đưa vào list đã tạo
			Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
			int count=0;
			// xử lý từng contour tìm được
			for (int i = 0; i < contours.size(); i++) {
				MatOfPoint src = new MatOfPoint(contours.get(i));
				MatOfPoint2f dst = new MatOfPoint2f();  
				src.convertTo(dst, CvType.CV_32F);
				boundRect.add(Imgproc.minAreaRect(dst));
					if (boundRect.get(i).size.area() > 300) {
						Imgproc.putText(mat, Math.round(boundRect.get(i).angle)+"", new Point(boundRect.get(i).center.x,boundRect.get(i).center.y-30), 0 , 0.5 , new Scalar(255,255,255));
						Imgproc.putText(mat, Math.round(boundRect.get(i).size.area())+"", new Point(boundRect.get(i).center.x,boundRect.get(i).center.y+50), 0 , 0.5 , new Scalar(255,255,255));
						Imgproc.putText(mat, Math.round(boundRect.get(i).center.x)+"", new Point(boundRect.get(i).center.x,boundRect.get(i).center.y+80), 0 , 0.5 , new Scalar(255,255,255));
					}
				}
			for (int i = 0; i < boundRect.size() - 1; i++) {
				// tạo một bounding box hình chữ nhật bao quanh contour
				// nếu kích cỡ của contour lớn hơn 1 giá trị
				// dùng để loại các vật thể nhiễu có thể lẫn vào
				if (boundRect.get(i).size.area() > 300) {
					

//					MatOfPoint src = new MatOfPoint(contours.get(i));
//					MatOfPoint2f dst = new MatOfPoint2f();  
//					src.convertTo(dst, CvType.CV_32F);
//					RotatedRect boundRect1 = Imgproc.minAreaRect(dst);
//					if (contours.size()>1) {
//						MatOfPoint src1 = new MatOfPoint(contours.get(i+1));
//						MatOfPoint2f dst1 = new MatOfPoint2f();  
//						src1.convertTo(dst1, CvType.CV_32F);
//						RotatedRect boundRect2 = Imgproc.minAreaRect(dst1);
						if (boundRect.get(i).angle>boundRect.get(i+1).angle && Math.round(boundRect.get(i).angle)!=-90 && Math.round(boundRect.get(i+1).angle)!=-90) {
							centerX = Math.round((boundRect.get(i).center.x+boundRect.get(i+1).center.x)/2);
							centerY = Math.round((boundRect.get(i).center.y+boundRect.get(i+1).center.y)/2);
							Imgproc.putText(mat,"here", new Point(centerX,centerY), 0 , 0.5 , new Scalar(255,255,255));					

					}
					// tính tọa độ trọng tâm của bounding box
				}
			}
			
//			if (centerX < 140) {
//				RobotMap.ss_drivebase.m_left.set(-0.15);
//				RobotMap.ss_drivebase.m_right.set(-0.15);
//			} else if (centerX > 180) {
//				RobotMap.ss_drivebase.m_left.set(0.15);
//				RobotMap.ss_drivebase.m_right.set(0.15);
//			} else {
//				RobotMap.ss_drivebase.m_left.set(0);
//				RobotMap.ss_drivebase.m_right.set(0);
//			}
			SmartDashboard.putString("Coordinates", centerX + ", " + centerY);
//			RobotMap.ss_drivebase.turnPID(160, 0.013, 0, 0.027);

			// đưa các giá trị ra dashboard

			// đưa output ra dashboard để tiện theo dõi
			outputStream.putFrame(mat);
		}
		});
		m_visionThread.setDaemon(true);
		m_visionThread.start();
		// Màu của Vision Target theo tính toán của Đỗ Hoàng Minh:
		// R: 70, RE: 20
		// G: 35, GE: 10
		// B: 245, BE: 10
	}
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public void TurnToCenter() {
		if (centerX<140){
			RobotMap.m_left.set(-0.25);
			RobotMap.m_right.set(-0.25);
		}
		else if (centerX>180) {
			RobotMap.m_left.set(0.25);
			RobotMap.m_right.set(0.25);
		}
		else {
			RobotMap.m_left.set(0);
			RobotMap.m_right.set(0);
		}
	}
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub

	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public double pidGet() {
		// TODO Auto-generated method stub
		return centerX;
	}
}