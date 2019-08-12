/* Daphne Barretto, Lead Software Developer
 * FRC Team 2614, MARS
 *
 * Debugging:
 * - Fix Duplicate Series Exceptions
 * - Improve GUI adaptability
 * - Remove coordTextFields and just use waypointTable because redundancy
 * - Allow directory change (use the setPathsFolder function that doesn't work yet; change csvFile variable in save btn code)
 * - Alert if no name when saving / loading (currently just does not save)
 * - Alert if send failed
 * 
 * Off-Season/Future Projects:
 * - Swerve Modifier
 * - Close window with Command+W
 * - Can I Bootstrap this?
 */

/**
 * @author Daphne Barretto
 */

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.net.SocketException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.apache.commons.net.ftp.FTP;
import org.apache.commons.net.ftp.FTPClient;
import org.apache.commons.net.ftp.FTPConnectionClosedException;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Trajectory.FitMethod;
import jaci.pathfinder.modifiers.TankModifier;
//import jaci.pathfinder.Pathfinder.GenerationException;

import javafx.application.Application;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.geometry.HPos;
import javafx.geometry.Insets;
import javafx.geometry.VPos;
import javafx.scene.Scene;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.ScatterChart;
import javafx.scene.chart.XYChart;
import javafx.scene.chart.XYChart.Data;
import javafx.scene.control.Alert;
import javafx.scene.control.Button;
import javafx.scene.control.ButtonType;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Menu;
import javafx.scene.control.MenuBar;
import javafx.scene.control.MenuItem;
import javafx.scene.control.Separator;
import javafx.scene.control.TextField;
import javafx.scene.control.ToolBar;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyCodeCombination;
import javafx.scene.input.KeyCombination;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.layout.ColumnConstraints;
import javafx.scene.layout.GridPane;
//import javafx.scene.layout.RowConstraints;
import javafx.scene.text.Text;
import javafx.stage.Stage;
import javafx.scene.layout.VBox;

public class PathPlannerGUI extends Application{

//	Global Variables -------------------------------------------------------------------------------------------------------
	
	//	Point Variables
	int pointCount = 0;
	Trajectory currentTrajectory;
	Trajectory currentLeftTrajectory;
	Trajectory currentRightTrajectory;
	boolean closeAlerts = false;
	boolean currentSubmitExists = false;
	ArrayList<ArrayList<Float>> waypoints = new ArrayList<ArrayList<Float>>();
	Double[] robotRotationalVelocityArray;
	
	//	Path Options Variables; initializes to default
	boolean robotDirectionIsForwards = true;
	FitMethod configFit = Trajectory.FitMethod.HERMITE_CUBIC;
	int configSamples = Trajectory.Config.SAMPLES_HIGH;
	double configDt = 0.04;
	double configMaxVelocity = 1.5;
	double configMaxAcceleration = 2.0;
	double configMaxJerk = 60.0;
	double configWheelbaseWidth = 0.5;
	
	//	Field Variables
	final double fieldLengthFt = 54;		// Note: Actual field is ~54.333 feet; simplified for ease of use
    final int fieldWidthFt = 27;
	
    //	File Variables
    String pathsFolderString = System.getProperty("user.home") + "/Desktop/Autonomous-Paths/";
	File pathsFolder = new File(pathsFolderString); 	// Default Paths Folder
	File[] pathsInFolder = pathsFolder.listFiles();
	
	double stageHeight;
	int pathOptionsWidth = 145;
	
	Double estimatedRunTime = 0.0;	//	in seconds
	
//	JavaFX Object Variables	-----------------------------------------------------------------------------------------------
	
	MenuBar menuBar = new MenuBar();
	
	GridPane superPane = new GridPane();
		ToolBar toolBar = new ToolBar();
			Button saveFileBtn = new Button("Save File");
			Button loadFileBtn = new Button("Load File");
			Button newPathBtn = new Button("New Path");
			final ComboBox<String> fileNameComboBox = new ComboBox<String>();
			Button sendFileBtn = new Button("Send File");
			Button sendAllFilesBtn = new Button("Send All Files");
			TextField robotAddressTF = new TextField();
		GridPane pane = new GridPane();
			GridPane waypointTableLabeled = new GridPane();
				GridPane waypointLabels = new GridPane();
				GridPane waypointTable = new GridPane();
					ArrayList<ArrayList<TextField>> coordTextFields = new ArrayList<ArrayList<TextField>>();
					ArrayList<Button> deleteBtns = new ArrayList<Button>();
			GridPane pathOptions = new GridPane();
				Button addPointBtn = new Button("Add Point");
				Button submitBtn = new Button("Submit");
				ObservableList<String> robotDirectionOptions = FXCollections.observableArrayList("Forwards", "Backwards");
					final ComboBox<String> robotDirectionComboBox = new ComboBox<String>(robotDirectionOptions);		
				ObservableList<String> fitOptions = FXCollections.observableArrayList("Cubic", "Quintic");
					final ComboBox<String> fitComboBox = new ComboBox<String>(fitOptions);		
				ObservableList<String> samplesOptions = FXCollections.observableArrayList("Fast", "Low", "High");
					final ComboBox<String> samplesComboBox = new ComboBox<String>(samplesOptions);
				TextField dtInput = new TextField();
				TextField maxVelocityInput = new TextField();
				TextField maxAccelerationInput = new TextField();
				TextField maxJerkInput = new TextField();
				TextField wheelbaseWidthInput = new TextField();
				Button horizontalMirrorPathBtn = new Button("Horizontal Mirror");
				Button verticalMirrorPathBtn = new Button("Vertical Mirror");
			XYChart.Series<Number, Number> xySeries = new XYChart.Series<Number, Number>();
			    	final NumberAxis mapXAxis = new NumberAxis();
			    	final NumberAxis mapYAxis = new NumberAxis();
	        		ScatterChart<Number,Number> XYMap = new ScatterChart<Number,Number>(mapXAxis,mapYAxis);
	        	XYChart.Series<Number, Number> velocitySeries = new XYChart.Series<Number, Number>();
	        	XYChart.Series<Number, Number> rotationalVelocitySeries = new XYChart.Series<Number, Number>();
		        	final NumberAxis velocityXAxis = new NumberAxis();
		        	final NumberAxis velocityYAxis = new NumberAxis();
		        	final ScatterChart<Number,Number> velocityScatterChart = new ScatterChart<Number,Number>(velocityXAxis,velocityYAxis);
	
	VBox vBox = new VBox(menuBar);
		        	
	Scene scene = new Scene(superPane);	//	Scene for entire applications

	 final Menu menu1 = new Menu("File");
	 final MenuItem saveItem = new MenuItem("Save");
	 final Menu menu2 = new Menu("Options");
	 final Menu menu3 = new Menu("Help");
	 final MenuItem documentation = new MenuItem("Documention");
	
//	Main Method	---------------------------------------------------------------------------------------------------------------
	
	public static void main(String[] args){
		Application.launch(args);
		
	}
	
//	Start Method		-----------------------------------------------------------------------------------------------------------	

	@Override
	public void start(Stage primaryStage){
		
		setUpSuperPane();
		setUpBtnListeners();
		newPathBtn.fire();
		
		//	Set up primaryStage
		primaryStage.setTitle("Path Planner 2018");
		primaryStage.setScene(scene);
		primaryStage.show();
		
		primaryStage.setX(0);
		primaryStage.setY(0);
		
		stageHeight = primaryStage.getHeight();
		
		reformatPathOptions();
		
		//	Set up scene
		scene.getStylesheets().add("main.css");	// main.css (located in src) to scene
		
	}
	
	/**
	 * Sets up the Super Pane, the root node of the scene graph. It contains all other Application nodes.
	 */
	public void setUpSuperPane() {
		
		setUpMenuBar();
		superPane.add(vBox, 0, 0);
		
		setUpToolBar();
		superPane.add(toolBar, 0, 1);
		
		setUpPane();
		superPane.add(pane, 0, 2);
		
	}
	
	/**
	 * This creates the MenuBar
	 */
	public void setUpMenuBar() {
		
		 
		 menu1.getItems().add(saveItem);
		 
		 setUpSaveItem();
		 saveItem.setAccelerator(new KeyCodeCombination(KeyCode.S, KeyCombination.META_DOWN));
		 
		 menu3.getItems().add(documentation);
		 documentation.setOnAction(new EventHandler<ActionEvent>() {
		     @Override
		     public void handle(ActionEvent event) {
		         getHostServices().showDocument("https://docs.google.com/document/d/1dPLZ9UW4Aaf6HdDpsfiBz-knBCVvO26UnwpuHS8aBWw/edit?usp=sharing");
		     }
		 });
		 documentation.setAccelerator(new KeyCodeCombination(KeyCode.F, KeyCombination.META_DOWN));
		 
		 menuBar.getMenus().addAll(menu1, menu2, menu3);
		
	}
	
	public void setUpToolBar() {
		
		int toolBarOptionWidth = 100;
		
		saveFileBtn.setPrefWidth(toolBarOptionWidth);
		
		loadFileBtn.setPrefWidth(toolBarOptionWidth);
		
		newPathBtn.setPrefWidth(toolBarOptionWidth);
		
		reloadFileNames();
		fileNameComboBox.setPromptText("File Name");
		fileNameComboBox.setEditable(true);
		fileNameComboBox.setPrefWidth(toolBarOptionWidth * 2);
	    
		sendFileBtn.setPrefWidth(toolBarOptionWidth * 1.2);
		
		sendAllFilesBtn.setPrefWidth(toolBarOptionWidth * 1.2);
		
		robotAddressTF.setPromptText("Team #");
		robotAddressTF.setPrefWidth(toolBarOptionWidth);
		robotAddressTF.setText("2614");
		
	    toolBar.prefWidthProperty().bind(scene.widthProperty());
		
	    toolBar.getItems().addAll(
	            saveFileBtn,
	            loadFileBtn,
	            newPathBtn,
	            fileNameComboBox,
	            new Separator(),
	            sendFileBtn,
	            sendAllFilesBtn,
	            new Text("Team Number:"),
	            robotAddressTF
	        );
		
	}
	
	public void setUpPane() {
		
		int column1PercentWidth = 30;
		ColumnConstraints column1 = new ColumnConstraints();
		column1.setPercentWidth(column1PercentWidth);
		ColumnConstraints column2 = new ColumnConstraints();
		column2.setPercentWidth(100 - column1PercentWidth);
		pane.getColumnConstraints().addAll(column1, column2);
		
//		int row1PercentHeight = 60;
//		RowConstraints row1 = new RowConstraints();
//		row1.setPercentHeight(row1PercentHeight);
//		RowConstraints row2 = new RowConstraints();
//		row2.setPercentHeight(100 - row1PercentHeight);
//		pane.getRowConstraints().addAll(row1, row2);
		
		// Pane (0,0) - WaypointTable; Pane (0, 1) - PathOptions; Pane (1, 0) - XYMap; Pane (1, 1) - XVelocityGraph
		pane.setPadding(new Insets(0));
		
		setUpWaypointTable();
		pane.add(waypointTableLabeled, 0, 0);	//to remove labels, switch this to waypointTable
		
		setUpPathOptions();
		pane.add(pathOptions, 0, 1);
		
		setUpXYMap();
        pane.add(XYMap, 1, 0);
		
		setUpXVelocityGraph();
        pane.add(velocityScatterChart, 1, 1);
		
	}
	
	public void setUpWaypointTable() {
		
		//	Set up Waypoint Table
		
		GridPane.setColumnSpan(waypointTable, 6);
		GridPane.setRowSpan(waypointTable, 2);
		waypointTable.setPadding(new Insets(5));
		waypointTable.setVgap(5);
		waypointTable.setHgap(5);
		
		waypointLabels.prefWidthProperty().bind(waypointTable.widthProperty());
		waypointLabels.setPadding(new Insets(5));
		waypointLabels.setVgap(5);
		waypointLabels.setHgap(5);
		
		Text xPosLabel = new Text("X Position");
		Text yPosLabel = new Text("Y Position");
		Text angleLabel = new Text("Angle");
		
		waypointTable.add(xPosLabel, 0, 0, 2, 1);
		waypointTable.add(yPosLabel, 2, 0, 2, 1);
		waypointTable.add(angleLabel, 4, 0, 1, 1);
		
		GridPane.setHalignment(xPosLabel, HPos.CENTER);
		GridPane.setHalignment(yPosLabel, HPos.CENTER);
		GridPane.setHalignment(angleLabel, HPos.CENTER);
		
		waypointTableLabeled.add(waypointLabels, 0, 0);
		waypointTableLabeled.add(waypointTable, 0, 1);
		
	}
	
	public void setUpPathOptions() {
		
		//	Set up Path Options

		/**
         * Create a Trajectory Configuration
         * @param fit                   The fit method to use
         * @param samples               How many samples to use to refine the path (higher = smoother, lower = faster)
         * @param dt                    The time delta between points (in seconds)
         * @param max_velocity          The maximum velocity the body is capable of traveling at (in meters per second)
         * @param max_acceleration      The maximum acceleration to use (in meters per second per second)
         * @param max_jerk              The maximum jerk (acceleration per second) to use
         */
		
		pathOptions.add(addPointBtn, 0, 0);
		pathOptions.add(submitBtn,  1,  0);
		pathOptions.add(horizontalMirrorPathBtn, 0, 1);
		pathOptions.add(verticalMirrorPathBtn, 1, 1);
		pathOptions.add(new Text("Robot Direction"), 0, 2);
		pathOptions.add(robotDirectionComboBox, 1, 2);
		pathOptions.add(new Text("Fit Method"), 0, 3);
		pathOptions.add(fitComboBox, 1, 3);
		pathOptions.add(new Text("Sample Count"), 0, 4);
		pathOptions.add(samplesComboBox, 1, 4);
		pathOptions.add(new Text("Delta Time (ms)"), 0, 5);
		pathOptions.add(dtInput, 1, 5);
		pathOptions.add(new Text("Max Velocity (m/s)"), 0, 6);
		pathOptions.add(maxVelocityInput, 1, 6);
		pathOptions.add(new Text("Max Acceleration"), 0, 7);
		pathOptions.add(maxAccelerationInput, 1, 7);
		pathOptions.add(new Text("Max Jerk"), 0, 8);
		pathOptions.add(maxJerkInput, 1, 8);
		pathOptions.add(new Text("Wheelbase Width (m)"), 0, 9);
		pathOptions.add(wheelbaseWidthInput, 1, 9);
		
		pathOptions.setVgap(3);
		pathOptions.setHgap(5);
		
		pathOptions.setPadding(new Insets(5));
		GridPane.setValignment(pathOptions, VPos.BOTTOM);
		
		pathOptionsWidth = 145;
		
		addPointBtn.setPrefWidth(pathOptionsWidth);
		
		submitBtn.setPrefWidth(pathOptionsWidth);
		
		robotDirectionComboBox.getSelectionModel().selectFirst();
		robotDirectionComboBox.setPrefWidth(pathOptionsWidth);
		
		fitComboBox.getSelectionModel().selectFirst();
		fitComboBox.setPrefWidth(pathOptionsWidth);
		
		samplesComboBox.getSelectionModel().selectFirst();
		samplesComboBox.setPrefWidth(pathOptionsWidth);
		
		dtInput.setPromptText("dt (ms)");
		dtInput.setMaxWidth(pathOptionsWidth);
		dtInput.setText("40");

		maxVelocityInput.setPromptText("maxVelocity");
		maxVelocityInput.setMaxWidth(pathOptionsWidth);
		maxVelocityInput.setText("1.5");
		
		maxAccelerationInput.setPromptText("maxAcceleration");
		maxAccelerationInput.setMaxWidth(pathOptionsWidth);
		maxAccelerationInput.setText("2");
		
		maxJerkInput.setPromptText("maxJerk");
		maxJerkInput.setMaxWidth(pathOptionsWidth);
		maxJerkInput.setText("60");
		
		//The width (in meters) between the individual sides of the drivebase
		wheelbaseWidthInput.setPromptText("wheelbaseWidth");
		wheelbaseWidthInput.setMaxWidth(pathOptionsWidth);
		wheelbaseWidthInput.setText("0.5");
		
		horizontalMirrorPathBtn.setPrefWidth(pathOptionsWidth);
		
		verticalMirrorPathBtn.setPrefWidth(pathOptionsWidth);
		
	}

	public void setUpXYMap() {
		
		//	Set up XYMap
		
		//defining the axes
	    mapXAxis.setLabel("X Position (ft)");
	    mapYAxis.setLabel("Y Position (ft)");
	    mapXAxis.setAutoRanging(false);
	    mapXAxis.setLowerBound(0);
	    mapXAxis.setUpperBound(fieldLengthFt);
	    mapXAxis.setTickUnit(2);
	    mapYAxis.setAutoRanging(false);
	    mapYAxis.setLowerBound(0);
	    mapYAxis.setUpperBound(fieldWidthFt);
	    mapYAxis.setTickUnit(2);
	    XYMap.setTitle("X-Y Map");
	    XYMap.setLegendVisible(false);
//	    XYMap.setPrefSize(400*1.7, 400);
//	    XYMap.setPrefSize(fieldWidthFt, fieldLengthFt);
	    double multiplier = 12.51;
	    double multiplier2 = 14.75;
//	    multiplier = multiplier/2;
//	    multiplier2 = multiplier2/2;
	    XYMap.setMinSize(fieldLengthFt * multiplier, fieldWidthFt * multiplier2);
	    XYMap.setMaxSize(fieldLengthFt * multiplier, fieldWidthFt * multiplier2);
	    XYMap.setId("XYMapForwards");
	    //defining a series
	    xySeries.setName("Path");
		
	}
	
	public void setUpXVelocityGraph() {
		
		//	Set up XVelocity Graph
		
		//defining the axes
		velocityXAxis.setLabel("Time (s)");
		velocityYAxis.setLabel("V (m/s),  Rot V (radians/s)");
	    velocityXAxis.setAutoRanging(false);
	    velocityXAxis.setLowerBound(-0.1);
	    velocityXAxis.setUpperBound(4);
	    velocityXAxis.setTickUnit(2);
	    velocityYAxis.setAutoRanging(false);
	    if (maxVelocityInput.getText().equals("")) {
	    		velocityYAxis.setUpperBound(1.5);
	    		velocityYAxis.setLowerBound(-1.5);
	    } else {
	    		velocityYAxis.setUpperBound(Double.parseDouble(maxVelocityInput.getText()));
	    		velocityYAxis.setLowerBound(-Double.parseDouble(maxVelocityInput.getText()));
	    }
	    velocityYAxis.setTickUnit(0.5);
	    velocityScatterChart.setTitle("Velocity-Time Graph");
	   
	    double multiplier = 12.51;
	    double multiplier2 = 11;
	    velocityScatterChart.setMinSize(fieldLengthFt * multiplier, fieldWidthFt * multiplier2);
	    velocityScatterChart.setMaxSize(fieldLengthFt * multiplier, fieldWidthFt * multiplier2);
	    //defining a series
	    velocitySeries.setName("Velocity");
	    rotationalVelocitySeries.setName("Rotational Velocity");
	    
	    velocityScatterChart.setLegendVisible(true);
	    velocityScatterChart.setId("Velocity-TimeGraph");
		
	}
	
//	Button Listeners Set Up Methods---------------------------------------------------------------------------------------------
	
	public void setUpBtnListeners() {
		
		setUpSaveFileBtn();
		setUpLoadFileBtn();
		setUpNewPathBtn();
		
		setUpSendFileBtn();
		setUpSendAllFilesBtn();
		
		setUpSubmitBtn();
		setUpAddPointBtn();	//	Contains listeners for delete point buttons
		setUpHorizontalMirrorPathBtn();
		setUpVerticalMirrorPathBtn();
		
	}
	
	public void setUpSaveItem() {
		 saveItem.setOnAction(new EventHandler<ActionEvent>() {
			 @Override
			 public void handle(ActionEvent event) {
				 
					System.out.println("Save");
					
					submitBtn.fire();
					
					String fileName = fileNameComboBox.getValue();
					if (fileName.equals("")){
						fileName = "defaultFileName";
					}
					if(fileName.contains(".csv"));
					else fileName += ".csv";
					
					String csvFile = pathsFolderString;
					FileWriter writer;
			        
			        pathsFolder= new File(csvFile);
			        if (!pathsFolder.exists()){
			        		pathsFolder.mkdir();
			        }
			        
			        csvFile += fileName;
			        
					try {
						writer = new FileWriter(csvFile);

					    DecimalFormat dfToThousandths = new DecimalFormat("###.###");
						
					    double initialX = 0;
					    double initialY = 0;
					    double initialAngle = 0;
					    
						for (int i = 0; i < currentRightTrajectory.length(); i++) {
							
						    Trajectory.Segment rightSeg = currentRightTrajectory.get(i);
						    Trajectory.Segment leftSeg = currentLeftTrajectory.get(i);
						    Trajectory.Segment seg = currentTrajectory.get(i);
						    
						    List<String> textLine = null;
						    String pointNum = Integer.toString(i + 1);
						    String leftVelocity = dfToThousandths.format(-leftSeg.velocity);
						    String rightVelocity = dfToThousandths.format(rightSeg.velocity);
						    String leftPosition = dfToThousandths.format(-leftSeg.position);
						    String rightPosition = dfToThousandths.format(rightSeg.position);
						    
						    if (!robotDirectionIsForwards) {
						    	
						    		System.out.println("Now Backwards");
						    	
						    		leftVelocity = dfToThousandths.format(rightSeg.velocity);
						    		rightVelocity = dfToThousandths.format(-leftSeg.velocity);
						    		leftPosition = dfToThousandths.format(rightSeg.position);
						    		rightPosition = dfToThousandths.format(-leftSeg.position);
						    }
						    
						    String waypointXFt = null;
						    String waypointXIn = null;
						    String waypointYFt = null;
						    String waypointYIn = null;
						    String waypointAngle = null;
						    
						    if (i < pointCount){
							    waypointXFt = coordTextFields.get(i).get(0).getText();
							    waypointXIn = coordTextFields.get(i).get(1).getText();
							    waypointYFt = coordTextFields.get(i).get(2).getText();
							    waypointYIn = coordTextFields.get(i).get(3).getText();
							    waypointAngle = coordTextFields.get(i).get(4).getText();
						    }
						    
						    if(i == 0) {
						    		initialX = seg.x;
						    		initialY = seg.y;
						    		initialAngle = 0 - Pathfinder.d2r(Double.parseDouble(waypointAngle));
						    }
						    
						    double currentX = seg.x - initialX;
						    double currentY = seg.y - initialY;
						    
						    double currentXPrime = currentX * Math.cos(initialAngle) - currentY * Math.sin(initialAngle);
						    double currentYPrime = currentX * Math.sin(initialAngle) + currentY * Math.cos(initialAngle);
						    
						    String robotX = dfToThousandths.format(currentXPrime);
						    String robotY = dfToThousandths.format(currentYPrime);
						        
						    String robotVelocity = dfToThousandths.format(seg.velocity);
						    
						    if(!robotDirectionIsForwards) {
						    		robotVelocity = dfToThousandths.format(-seg.velocity);
						    }
						    
						    String currentPathOption = "";
						    
						    switch (i) {
						    		case 0:
						    			currentPathOption = robotDirectionComboBox.getValue();
						    			break;
						    		case 1:
						    			currentPathOption = fitComboBox.getValue();
						    			break;
						    		case 2:
						    			currentPathOption = samplesComboBox.getValue();
						    			break;
						    		case 3:
						    			currentPathOption = dtInput.getText();
						    			break;
						    		case 4:
						    			currentPathOption = maxVelocityInput.getText();
						    			break;
						    		case 5:
						    			currentPathOption = maxAccelerationInput.getText();
						    			break;
						    		case 6:
						    			currentPathOption = maxJerkInput.getText();
						    			break;
						    		case 7:
						    			currentPathOption = wheelbaseWidthInput.getText();
						    			break;
						    		default:
						    			currentPathOption = " ";
						    			break;
						    }
						    
						    String robotRotationalVelocity = Double.toString(robotRotationalVelocityArray[i]);
						    
						    String elapsedTime = Double.toString(i * configDt);
						    
						    if (i < pointCount) {
						    		textLine = Arrays.asList(pointNum, leftVelocity, rightVelocity, leftPosition, rightPosition, elapsedTime,
						    				robotX, robotY, robotVelocity, robotRotationalVelocity,
						    				waypointXFt, waypointXIn, waypointYFt, waypointYIn, waypointAngle, "",
						    				currentPathOption);
						    } else {
						    		textLine = Arrays.asList(pointNum, leftVelocity, rightVelocity, leftPosition, rightPosition, elapsedTime,
					    				robotX, robotY, robotVelocity, robotRotationalVelocity,
					    				"", "", "", "", "", "",
					    				currentPathOption);
						    }
						    
						    CSVUtils.writeLine(writer, textLine, ',');
						    
						}
						writer.flush();
				        writer.close();
					} catch (IOException e1) {
						e1.printStackTrace();
					}
					
					reloadFileNames();
					
					fileNameComboBox.setValue(fileName);
				 
			 }
		 });
	}
	
	public void setUpSaveFileBtn() {
		
		saveFileBtn.setOnAction(e -> {
			
			System.out.println("Save");
			
			submitBtn.fire();
			
			String fileName = fileNameComboBox.getValue();
			if (fileName.equals("")){
				fileName = "defaultFileName";
			}
			if(fileName.contains(".csv"));
			else fileName += ".csv";
			
			String csvFile = pathsFolderString;
			FileWriter writer;
	        
	        pathsFolder= new File(csvFile);
	        if (!pathsFolder.exists()){
	        		pathsFolder.mkdir();
	        }
	        
	        csvFile += fileName;
	        
			try {
				writer = new FileWriter(csvFile);

			    DecimalFormat dfToThousandths = new DecimalFormat("###.###");
				
			    double initialX = 0;
			    double initialY = 0;
			    double initialAngle = 0;
			    
				for (int i = 0; i < currentRightTrajectory.length(); i++) {
					
				    Trajectory.Segment rightSeg = currentRightTrajectory.get(i);
				    Trajectory.Segment leftSeg = currentLeftTrajectory.get(i);
				    Trajectory.Segment seg = currentTrajectory.get(i);
				    
				    List<String> textLine = null;
				    String pointNum = Integer.toString(i + 1);
				    String leftVelocity = dfToThousandths.format(-leftSeg.velocity);
				    String rightVelocity = dfToThousandths.format(rightSeg.velocity);
				    String leftPosition = dfToThousandths.format(-leftSeg.position);
				    String rightPosition = dfToThousandths.format(rightSeg.position);
				    
				    if (!robotDirectionIsForwards) {
				    	
				    		System.out.println("Now Backwards");
				    	
				    		leftVelocity = dfToThousandths.format(rightSeg.velocity);
				    		rightVelocity = dfToThousandths.format(-leftSeg.velocity);
				    		leftPosition = dfToThousandths.format(rightSeg.position);
				    		rightPosition = dfToThousandths.format(-leftSeg.position);
				    }
				    
				    String waypointXFt = null;
				    String waypointXIn = null;
				    String waypointYFt = null;
				    String waypointYIn = null;
				    String waypointAngle = null;
				    
				    if (i < pointCount){
					    waypointXFt = coordTextFields.get(i).get(0).getText();
					    waypointXIn = coordTextFields.get(i).get(1).getText();
					    waypointYFt = coordTextFields.get(i).get(2).getText();
					    waypointYIn = coordTextFields.get(i).get(3).getText();
					    waypointAngle = coordTextFields.get(i).get(4).getText();
				    }
				    
				    if(i == 0) {
				    		initialX = seg.x;
				    		initialY = seg.y;
				    		initialAngle = 0 - Pathfinder.d2r(Double.parseDouble(waypointAngle));
				    }
				    
				    double currentX = seg.x - initialX;
				    double currentY = seg.y - initialY;
				    
				    double currentXPrime = currentX * Math.cos(initialAngle) - currentY * Math.sin(initialAngle);
				    double currentYPrime = currentX * Math.sin(initialAngle) + currentY * Math.cos(initialAngle);
				    
				    String robotX = dfToThousandths.format(currentXPrime);
				    String robotY = dfToThousandths.format(currentYPrime);
				        
				    String robotVelocity = dfToThousandths.format(seg.velocity);
				    
				    if(!robotDirectionIsForwards) {
				    		robotVelocity = dfToThousandths.format(-seg.velocity);
				    }
				    
				    String currentPathOption = "";
				    
				    switch (i) {
				    		case 0:
				    			currentPathOption = robotDirectionComboBox.getValue();
				    			break;
				    		case 1:
				    			currentPathOption = fitComboBox.getValue();
				    			break;
				    		case 2:
				    			currentPathOption = samplesComboBox.getValue();
				    			break;
				    		case 3:
				    			currentPathOption = dtInput.getText();
				    			break;
				    		case 4:
				    			currentPathOption = maxVelocityInput.getText();
				    			break;
				    		case 5:
				    			currentPathOption = maxAccelerationInput.getText();
				    			break;
				    		case 6:
				    			currentPathOption = maxJerkInput.getText();
				    			break;
				    		case 7:
				    			currentPathOption = wheelbaseWidthInput.getText();
				    			break;
				    		default:
				    			currentPathOption = " ";
				    			break;
				    }
				    
				    String robotRotationalVelocity = Double.toString(robotRotationalVelocityArray[i]);
				    
				    String elapsedTime = Double.toString(i * configDt);
				    
				    if (i < pointCount) {
				    		textLine = Arrays.asList(pointNum, leftVelocity, rightVelocity, leftPosition, rightPosition, elapsedTime,
				    				robotX, robotY, robotVelocity, robotRotationalVelocity,
				    				waypointXFt, waypointXIn, waypointYFt, waypointYIn, waypointAngle, "",
				    				currentPathOption);
				    } else {
				    		textLine = Arrays.asList(pointNum, leftVelocity, rightVelocity, leftPosition, rightPosition, elapsedTime,
			    				robotX, robotY, robotVelocity, robotRotationalVelocity,
			    				"", "", "", "", "", "",
			    				currentPathOption);
				    }
				    
				    CSVUtils.writeLine(writer, textLine, ',');
				    
				}
				writer.flush();
		        writer.close();
			} catch (IOException e1) {
				e1.printStackTrace();
			}
			
			reloadFileNames();
			
			fileNameComboBox.setValue(fileName);
			
			
		});
	}
	
	public void setUpLoadFileBtn() {
		
		loadFileBtn.setOnAction(e -> {
			
			String fileName = fileNameComboBox.getValue();
			
			newPathBtn.fire();
			
			if (fileName.equals("")){
				fileName = "defaultFileName";
			}
			
			if(!fileName.contains(".csv")) {
				fileName += ".csv";
			}
			
			String csvFile = pathsFolderString + fileName;
	        BufferedReader br = null;
	        String line = "";
	        String cvsSplitBy = ",";
	        int numLoadedPoints = 0;
	        String[][] loadedPoints = new String[50][5]; 	//Assumes max points in a path is 25
	        String[] pathOptions = new String[300];
	        int waypointsInPath = 0;
	        
	        try {

	            br = new BufferedReader(new FileReader(csvFile));
	            
	            while ((line = br.readLine()) != null) {

	                // use comma as separator
	                String[] point = line.split(cvsSplitBy);
	                
	                if (point[10].length() <= 0 && numLoadedPoints >= 8) {
	                		break;
	                }
	                
	                if (point[10].length() > 0) {
	                		
	                		waypointsInPath++;
	                	
		                for(int i = 0; i < 5; i++){
		                		loadedPoints[numLoadedPoints][i] = point[i + 10];
		                }
		                
		                System.out.println("Point [X Position (ft)=" + point[10] + " , X Position (in)=" + point[11] + " , Y Position (ft)=" + point[12] + " , Y Position (in)=" + point[13] + " , Angle (degrees)=" + point[14] + "]");
	
	                }
	                
	                if (numLoadedPoints < 8) {
	                		pathOptions[numLoadedPoints] = point[16];
	                }
	                
	                numLoadedPoints++;
	                
	            }

	        } catch (FileNotFoundException e1) {
	            e1.printStackTrace();
	        } catch (IOException e1) {
	            e1.printStackTrace();
	        } finally {
	            if (br != null) {
	                try {
	                    br.close();
	                } catch (IOException e1) {
	                    e1.printStackTrace();
	                }
	            }
	        }
	       
	        for(int i = 2; i < waypointsInPath; i++){
	        		addPointBtn.fire();
	        }
	        
	        for(int i = 0; i < waypointsInPath; i++) {
				for(int j = 0; j < 5; j++) {
					coordTextFields.get(i).get(j).setText(loadedPoints[i][j]);	
				}
			}
	        
//	        for(int i = 0; i < 8; i++) {
//	        		System.out.println("Path Option: " + pathOptions[i]);
//	        }
	        
	        //	Load File Settings
	        robotDirectionComboBox.setValue(pathOptions[0]);
	        fitComboBox.setValue(pathOptions[1]);
	        samplesComboBox.setValue(pathOptions[2]);
	        dtInput.setText(pathOptions[3]);
	        maxVelocityInput.setText(pathOptions[4]);
	        maxAccelerationInput.setText(pathOptions[5]);
	        maxJerkInput.setText(pathOptions[6]);
	        wheelbaseWidthInput.setText(pathOptions[7]);
	        
	        fileNameComboBox.setValue(fileName);
	        
	        submitBtn.fire();
	        
		});
		
	}
	
	public void setUpNewPathBtn() {
		
		newPathBtn.setOnAction(e -> {
			while(deleteBtns.size() > 0) {
				System.out.println("deleteBtns index to fire: " + (pointCount - 1));
				deleteBtns.get(pointCount - 1).fire();
				System.out.println("deleteBtns.size(): " + deleteBtns.size());
			}
			addPointBtn.fire();
			addPointBtn.fire();
			
			if (currentSubmitExists) {
				currentTrajectory = null;
				currentLeftTrajectory = null;
				currentRightTrajectory = null;
				waypoints = new ArrayList<ArrayList<Float>>();
			}
			
			if (!XYMap.getData().isEmpty()) {
				XYMap.getData().remove(xySeries);
				while(!XYMap.getData().isEmpty()){
					System.out.println("Not empty yet!");
					XYMap.getData().remove(0);
				}
			}
			
			while(!xySeries.getData().isEmpty())
				xySeries.getData().remove(0);
			
			if (!velocityScatterChart.getData().isEmpty()) {
				velocityScatterChart.getData().remove(velocitySeries);
				while(!velocityScatterChart.getData().isEmpty()){
					System.out.println("Not empty yet!");
					velocityScatterChart.getData().remove(0);
				}
			}
			
			while(!velocitySeries.getData().isEmpty())
				velocitySeries.getData().remove(0);
			
			fileNameComboBox.getSelectionModel().clearSelection();
			
		});
		
	}	
	
	public void setUpSendFileBtn() {
		
		sendFileBtn.setOnAction(e -> {
			
			saveFileBtn.fire();	//Ensures file to send is saved
			
			String robotIP = "roboRIO-" + robotAddressTF.getText() + "-FRC.local";
			
			String fileNameToUpload = fileNameComboBox.getValue();
			if (fileNameToUpload.equals("")){
				fileNameToUpload = "defaultFileName";
			}
			if(fileNameToUpload.contains(".csv"));
			else fileNameToUpload += ".csv";
			
			sendPathFileToRobot(robotIP, fileNameToUpload);
			
		});
		
	}

	public void setUpSendAllFilesBtn() {
		
		sendAllFilesBtn.setOnAction(e -> {

			saveFileBtn.fire();	//Ensures file to send is saved
			
			String robotIP = "roboRIO-" + robotAddressTF.getText() + "-FRC.local";
			
			reloadFileNames();
		    
		    for(int i = 0; i < pathsInFolder.length; i++) {
		    		sendPathFileToRobot(robotIP, pathsInFolder[i].getName());
		    }
		    		
		});
		
	}
	
public void setUpSubmitBtn() {
		
		submitBtn.setOnAction(e -> {
			
			if (currentSubmitExists) {
				currentTrajectory = null;
				currentLeftTrajectory = null;
				currentRightTrajectory = null;
				waypoints = new ArrayList<ArrayList<Float>>();
			}
			
			currentSubmitExists = true;
			
			System.out.println("submit");
			
			//Loop points to check validity
			for(int i = 0; i < pointCount; i++){
				for(int j = 0; j < 5; j++){
					try {
						Float.parseFloat(coordTextFields.get(i).get(j).getText());
					} catch(NumberFormatException e1){
						Alert alert = new Alert(AlertType.ERROR);
						System.out.println("Catch");
						alert.setTitle("Error: NumberFormatException");
						alert.setHeaderText("Invalid Point Input");
						alert.setContentText("Check your points for blanks and invalid characters.");
						
						alert.showAndWait().ifPresent(response -> {
							if (response == ButtonType.OK) {
								alert.close();
								closeAlerts = true;
							}
						});
						
						if(closeAlerts)
							return;
					}
				}
			}
			
			for(int i = 0; i < pointCount; i++) {	//loop points
				ArrayList<Float> currentPoint = new ArrayList<Float>(3);
				
				for(int j = 0; j < 5; j += 2) {		//loop x (feet), y (feet), angle
					String currentNum = coordTextFields.get(i).get(j).getText();
						if(j == 0 || j == 2){
							
							//add inches column
							String currentInString = coordTextFields.get(i).get(j+1).getText();
							Float currentInFloat = Float.parseFloat(currentInString);
							Float dToAddFt = currentInFloat / 12;
							Float newNum = Float.parseFloat(currentNum) + dToAddFt;
							//convert to meters
							Float currentNumM = (float) (newNum * 0.3048);
							
							currentNum = Float.toString(currentNumM);
							currentPoint.add(currentNumM);
						} else if(j == 4){
							//convert degrees to radians
							
							coordTextFields.get(i).get(j).setText(Double.toString(Double.parseDouble(currentNum) % 360));
							
							currentNum = Double.toString(Pathfinder.d2r(Double.parseDouble(currentNum)));
							currentPoint.add(Float.parseFloat(currentNum));
						}
						
				}
				
				for(int count = 0; count < currentPoint.size(); count++) {
					System.out.print(currentPoint.get(count) + "   ");
				}
				System.out.println();
				
				waypoints.add(currentPoint);
			}

			if (robotDirectionComboBox.getValue() == "Backwards") {
				robotDirectionIsForwards = false;
				System.out.println("Robot is moving backwards");
				XYMap.setId("XYMapBackwards");
			} else {
				robotDirectionIsForwards = true;
				System.out.println("Robot is moving forwards");
				XYMap.setId("XYMapForwards");
			}
			
			//check if enough points
			if(waypoints.size() < 2){
				Alert alert = new Alert(AlertType.ERROR);
				alert.setTitle("Error: Path Cannot Be Generated");
				alert.setHeaderText("Not Enough Points");
				alert.setContentText("At least two points are needed to generate a path.");
				
				alert.showAndWait().ifPresent(response -> {
					if (response == ButtonType.OK) {
						alert.close();
						closeAlerts = true;
					}
				});
				
				if(closeAlerts){
					closeAlerts = false;
					return;
				}
			}
			
			Waypoint[] pointsArray = new Waypoint[pointCount];
			for(int k = 0; k < pointCount; k++) {
				Waypoint currentWaypoint = new Waypoint(waypoints.get(k).get(0), waypoints.get(k).get(1), waypoints.get(k).get(2));
				pointsArray[k] = currentWaypoint;
			}
			
			if(fitComboBox.getValue() == "Cubic")
				configFit = Trajectory.FitMethod.HERMITE_CUBIC;
			else if(fitComboBox.getValue() == "Quintic")
				configFit = Trajectory.FitMethod.HERMITE_QUINTIC;
			
			if(samplesComboBox.getValue() == "Fast")
				configSamples = Trajectory.Config.SAMPLES_FAST;
			else if(samplesComboBox.getValue() == "High")
				configSamples = Trajectory.Config.SAMPLES_HIGH;
			else if(samplesComboBox.getValue() == "Low")
				configSamples = Trajectory.Config.SAMPLES_LOW;
			
			try{
				if(!dtInput.getText().equals(""))
					configDt = Double.parseDouble(dtInput.getText()) /1000;
				if(!maxVelocityInput.getText().equals(""))
					configMaxVelocity = Double.parseDouble(maxVelocityInput.getText());
				if(!maxAccelerationInput.getText().equals(""))
					configMaxAcceleration = Double.parseDouble(maxAccelerationInput.getText());
				if(!maxJerkInput.getText().equals(""))
					configMaxJerk = Double.parseDouble(maxJerkInput.getText());
				if(!wheelbaseWidthInput.getText().equals(""))
					configWheelbaseWidth = Double.parseDouble(wheelbaseWidthInput.getText());
			} catch (NumberFormatException f){
				Alert alert = new Alert(AlertType.ERROR);
				System.out.println("Catch");
				alert.setTitle("Error: NumberFormatException");
				alert.setHeaderText("Invalid Config Input");
				alert.setContentText("Check for invalid config settings");
				
				alert.showAndWait().ifPresent(response -> {
					if (response == ButtonType.OK) {
						alert.close();
						closeAlerts = true;
					}
				});
				
				if(closeAlerts)
					return;
			}

			Trajectory.Config config = new Trajectory.Config(configFit, configSamples, configDt, configMaxVelocity, configMaxAcceleration, configMaxJerk);
			currentTrajectory = Pathfinder.generate(pointsArray, config);
		
			TankModifier modifier = new TankModifier(currentTrajectory).modify(configWheelbaseWidth);
			currentLeftTrajectory = modifier.getLeftTrajectory();
			currentRightTrajectory = modifier.getRightTrajectory();
			
			if (!XYMap.getData().isEmpty()) {
				XYMap.getData().remove(xySeries);
				while(!XYMap.getData().isEmpty()){
					System.out.println("Not empty yet!");
					XYMap.getData().remove(0);
				}
			}
			
			while(!xySeries.getData().isEmpty())
				xySeries.getData().remove(0);
			
//			System.out.println("Before: " + xySeries.getData().isEmpty());
			
			XYMap.getData().clear();
			xySeries.getData().clear();
			
//			System.out.println("After: " + xySeries.getData().isEmpty());
			
			if (!velocityScatterChart.getData().isEmpty()) {
				velocityScatterChart.getData().remove(velocitySeries);
				while(!velocityScatterChart.getData().isEmpty()){
					System.out.println("Not empty yet!");
					velocityScatterChart.getData().remove(0);
				}
			}
			
			while(!velocitySeries.getData().isEmpty())
				velocitySeries.getData().remove(0);
			
			while(!rotationalVelocitySeries.getData().isEmpty())
				rotationalVelocitySeries.getData().remove(0);
			
//			System.out.println("Before: " + velocitySeries.getData().isEmpty());
			
			velocityScatterChart.getData().clear();
			velocitySeries.getData().clear();
			rotationalVelocitySeries.getData().clear();
			
//			System.out.println("After: " + velocitySeries.getData().isEmpty());
			
//			double lowestX = 100, highestX = -100;
			
			Double previousRobotRotationalVelocityDouble = 0.0;
			Double previousRobotX = 0.0;
		    Double previousRobotY = 0.0;
		    Double startingRobotTheta = 0.0;
		    DecimalFormat dfToThousandths = new DecimalFormat("###.###");
			
		    Double biggestVelocity = 0.0;
		    Double lowestVelocity = 0.0;
		    
		    robotRotationalVelocityArray = new Double[currentTrajectory.length()];
		    
			for (int i = 0; i < currentTrajectory.length(); i++) {
			    Trajectory.Segment seg = currentTrajectory.get(i);
			    
//			    System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", 
//			        seg.dt, seg.x, seg.y, seg.position, seg.velocity, 
//			            seg.acceleration, seg.jerk, seg.heading);
			    
			    if (i == 0) {
					previousRobotX = seg.x;
					previousRobotY = seg.y;
					startingRobotTheta = Double.parseDouble(dfToThousandths.format(Math.toRadians(Double.parseDouble(coordTextFields.get(0).get(4).getText()))));	//	Grabbing the waypoint angle of first point
				}
			    
			    Double dtInSeconds = Double.parseDouble(dtInput.getText())/1000;
			    Double deltaX = seg.x - previousRobotX;
			    Double deltaY = seg.y - previousRobotY;
			    Double radiansToTurn;
			    if (!(Math.abs(deltaX) < 0.000001))
			    		radiansToTurn = Math.atan(deltaY/deltaX) - startingRobotTheta;
			    else
			    		radiansToTurn = 0.0;
			    
//			    System.out.println("deltaX = " + deltaX + "deltaY = " + dfToThousandths.format(deltaY) + "radiansToTurn = " + dfToThousandths.format(radiansToTurn) + "startingRobotTheta = " + startingRobotTheta);
//			    System.out.println("radiansToTurn = " + dfToThousandths.format(radiansToTurn) + "\tstartingRobotTheta = " + startingRobotTheta);
			    
			    radiansToTurn = Double.parseDouble(dfToThousandths.format(radiansToTurn));
			    
			    Double robotRotationalVelocityDouble = radiansToTurn/dtInSeconds;
			    
			    if (Math.abs(robotRotationalVelocityDouble) > 10)
			    		robotRotationalVelocityDouble = previousRobotRotationalVelocityDouble;
			    
			    robotRotationalVelocityArray[i] = robotRotationalVelocityDouble;
			    
			    
//			    System.out.println("radiansToTurn = " + dfToThousandths.format(radiansToTurn) + "\trobotRotationalVelocityDouble = " + dfToThousandths.format(robotRotationalVelocityDouble) + "\tstartingRobotTheta = " + startingRobotTheta);
			    
			    xySeries.getData().add(new Data<Number, Number>(seg.x / 0.3048, seg.y / 0.3048));
			    
			    Double velocity = seg.velocity;
			    
			    if (robotDirectionComboBox.getValue() == "Backwards") {
			    		velocity = -seg.velocity;
				}
			    
			    velocitySeries.getData().add(new Data<Number, Number>(i * configDt, velocity));
			    
		        rotationalVelocitySeries.getData().add(new Data<Number, Number>(i * configDt, robotRotationalVelocityDouble));
		        
		        if(velocity > biggestVelocity) {
		        		biggestVelocity = velocity;
		        } else if (velocity < lowestVelocity) {
		        		lowestVelocity = velocity;
		        }
		        
		        if(robotRotationalVelocityDouble > biggestVelocity) {
		        		biggestVelocity = robotRotationalVelocityDouble;
		        } else if (robotRotationalVelocityDouble < lowestVelocity) {
		        		lowestVelocity = robotRotationalVelocityDouble;
		        }
		        
			    previousRobotX = seg.x;
			    previousRobotY = seg.y;
			    previousRobotRotationalVelocityDouble = robotRotationalVelocityDouble;
			    startingRobotTheta += Double.parseDouble(dfToThousandths.format(radiansToTurn));
		        
			}
			
			XYMap.getData().add(xySeries);
			velocityScatterChart.getData().add(velocitySeries);
			velocityScatterChart.getData().add(rotationalVelocitySeries);	
			
			//defining the axes
			velocityXAxis.setUpperBound(Math.round(currentTrajectory.length() * configDt) + 1);
//			velocityYAxis.setUpperBound(Double.parseDouble(maxVelocityInput.getText()) + 0.5);
//			velocityYAxis.setLowerBound(-(Double.parseDouble(maxVelocityInput.getText()) + 0.5));
			
			velocityYAxis.setUpperBound(Math.round(biggestVelocity) + 0.5);
			velocityYAxis.setLowerBound(Math.round(lowestVelocity) - 0.5);
			
			estimatedRunTime = (currentTrajectory.length() - 1) * configDt;
			System.out.println("estimatedRunTime = " + estimatedRunTime);
			
			DecimalFormat dfToHundredths = new DecimalFormat("###.##");
			velocityScatterChart.setTitle("Velocity-Time Graph\t\t\tRun Time: " + dfToHundredths.format(estimatedRunTime) + "s");
			
		});
		
	}
	
	public void setUpHorizontalMirrorPathBtn() {
		horizontalMirrorPathBtn.setOnAction(e -> {
			
			for(int i = 0; i < pointCount; i++) {
				Double initialXPosFt = Double.parseDouble(coordTextFields.get(i).get(0).getText());
				Double initialXPosIn = Double.parseDouble(coordTextFields.get(i).get(1).getText());
				Double initialAngle = Double.parseDouble(coordTextFields.get(i).get(4).getText());
				
				if (initialXPosIn > 0) {
					coordTextFields.get(i).get(1).setText(Double.toString(12 - initialXPosIn));
					coordTextFields.get(i).get(0).setText(Double.toString(fieldLengthFt - 1 - initialXPosFt));
				} else {
					coordTextFields.get(i).get(0).setText(Double.toString(fieldLengthFt - initialXPosFt));
				}
				
				coordTextFields.get(i).get(4).setText(Double.toString(180 - initialAngle));
				
			}
			
			submitBtn.fire();
				
		});
	}
	
	public void setUpVerticalMirrorPathBtn() {
		verticalMirrorPathBtn.setOnAction(e -> {
			
			for (int i = 0; i < pointCount; i++) {
				Double initialYPosFt = Double.parseDouble(coordTextFields.get(i).get(2).getText());
				Double initialYPosIn = Double.parseDouble(coordTextFields.get(i).get(3).getText());
				Double initialAngle = Double.parseDouble(coordTextFields.get(i).get(4).getText());
				
				if (initialYPosIn > 0) {
					coordTextFields.get(i).get(3).setText(Double.toString(12-initialYPosIn));
					coordTextFields.get(i).get(2).setText(Double.toString(fieldWidthFt - 1 - initialYPosFt));
				} else {
					coordTextFields.get(i).get(2).setText(Double.toString(fieldWidthFt - initialYPosFt));
				}
				
				if (Math.abs(initialAngle) > 0) {
					coordTextFields.get(i).get(4).setText(Double.toString(360 - initialAngle));
				}
			}
			
			
			
			submitBtn.fire();
			
		});
	}

	public void setUpAddPointBtn() {
		addPointBtn.setOnAction(e -> {
			pointCount++;
			System.out.println("add");
			
			ArrayList<TextField> currentRow = new ArrayList<TextField>(5);
			
			int WaypointTFWidth = 45;
			TextField tempX = new TextField();
			tempX.setPromptText("ft");
			tempX.setPrefWidth(WaypointTFWidth);
			TextField tempXIn = new TextField();
			tempXIn.setPromptText("in");
			tempXIn.setText("0");
			tempXIn.setPrefWidth(WaypointTFWidth);
			TextField tempY = new TextField();
			tempY.setPromptText("ft");
			tempY.setPrefWidth(WaypointTFWidth);
			TextField tempYIn = new TextField();
			tempYIn.setPromptText("in");
			tempYIn.setText("0");
			tempYIn.setPrefWidth(WaypointTFWidth);
			TextField tempAngle = new TextField();
			tempAngle.setPromptText("°");
			tempAngle.setPrefWidth(WaypointTFWidth + 10);
			
			currentRow.add(tempX);
			currentRow.add(tempXIn);
			currentRow.add(tempY);
			currentRow.add(tempYIn);
			currentRow.add(tempAngle);
			deleteBtns.add(new Button("X" + (pointCount)));
			deleteBtns.get(deleteBtns.size() - 1).setPrefWidth(WaypointTFWidth - 10);
			deleteBtns.get(deleteBtns.size()-1).setOnAction(f -> {
				
				System.out.println("delete");

				int rowToDeleteIndex = Integer.parseInt(((Button)f.getSource()).getText().substring(1)) - 1;
				int numOfRowsToMove = pointCount - rowToDeleteIndex - 1;
				
				System.out.println("numOfRowsToMove: " + numOfRowsToMove);
				String[][] newCoordTextFieldValues = new String[numOfRowsToMove][5];
				
				//	Records the values of all TextFields below deleted row
				int count = 0;
				for (int i = rowToDeleteIndex + 1; i < pointCount; i++) {
					for (int j = 0; j < 5; j++) {
						System.out.print(coordTextFields.get(i).get(j).getText() + "\t");
						newCoordTextFieldValues[count][j] = coordTextFields.get(i).get(j).getText();
					}
					count++;
					System.out.println();
				}
				
				//	Moves up the values from the TextFields below the deleted row
				count = 0;
				for (int i = rowToDeleteIndex; i < pointCount - 1; i++) {
					for (int j = 0; j < 5; j++) {
//						System.out.println("i = " + i + "; j = " + j);
						coordTextFields.get(i).get(j).setText(newCoordTextFieldValues[count][j]);
					}
					count++;
//					System.out.println();
				}
				
				pointCount--;
				
				//	Deletes the last row of children of the waypointTable
				waypointTable.getChildren().remove(pointCount * 6 + 3);
				waypointTable.getChildren().remove(pointCount * 6 + 3);
				waypointTable.getChildren().remove(pointCount * 6 + 3);
				waypointTable.getChildren().remove(pointCount * 6 + 3);
				waypointTable.getChildren().remove(pointCount * 6 + 3);
				waypointTable.getChildren().remove(pointCount * 6 + 3);
				
				deleteBtns.remove(deleteBtns.size() - 1);
				
				coordTextFields.remove(coordTextFields.size() - 1);
					
			});
			
			System.out.println("pointCount: " + (pointCount));
			waypointTable.add(currentRow.get(0), 0, pointCount);
			waypointTable.add(currentRow.get(1), 1, pointCount);
			waypointTable.add(currentRow.get(2), 2, pointCount);
			waypointTable.add(currentRow.get(3), 3, pointCount);
			waypointTable.add(currentRow.get(4), 4, pointCount);
			waypointTable.add(deleteBtns.get(deleteBtns.size() - 1), 5, pointCount);
			
			coordTextFields.add(currentRow);
		});
	}
	
	public void showServerReply(FTPClient ftpClient) {
	// Prints the server reply; use after different commands to check success
		
		String[] replies = ftpClient.getReplyStrings();
	    if (replies != null && replies.length > 0) {
	        for (String aReply : replies) {
	            System.out.println("SERVER: " + aReply);
	        }
	    }
	}
	
	public void sendPathFileToRobot(String robotIP, String fileNameToUpload) {
		
		String IP = robotIP;	// i.e. roboRIO-2614-FRC.local
	    int port = 21;
	    String user = "anonymous";
	    String pass = "";
	    FTPClient ftpClient = new FTPClient();
		
	    try {
	    	
	    	// connect and login
	    	ftpClient.connect(IP, port);
	    	showServerReply(ftpClient);
	    	ftpClient.login(user, pass);
			showServerReply(ftpClient);
			
			// setup
			ftpClient.enterLocalPassiveMode();
			ftpClient.setFileType(FTP.BINARY_FILE_TYPE);
			showServerReply(ftpClient);
			
			// variables for the local and remote files
			String localFileName = fileNameToUpload; // i.e. 4mStraightNew.csv
			String localFileDir = System.getProperty("user.home") + "/Desktop/Autonomous-Paths/"; // directory on the computer where the file to upload is located
			File localFile = new File(localFileDir + localFileName);
			String remoteFileDir = "/home/lvuser/natinst/bin/Autonomous-Paths/";	// directory on the rio to place the file
			String remoteFile = remoteFileDir + localFileName;
			
			// prints the list of files in rioDir before uploading the new file
			String[] names = ftpClient.listNames(remoteFileDir);
			for(int i = 0; i < names.length; i ++){
//				System.out.println(names[i]);	
			}
			showServerReply(ftpClient);
            
            // uploads file; will rewrite existing file of the same name
            InputStream inputStream = new FileInputStream(localFile);
            boolean uploadSuccess = ftpClient.storeFile(remoteFile, inputStream);
            inputStream.close();
            if (uploadSuccess) {
            		System.out.println("file upload success");
            } else {
            		System.out.println("file upload fail");
            }

			// prints the list of files in rioDir after uploading the new file
			names = ftpClient.listNames(remoteFileDir);
//			for(int i = 0; i < names.length; i ++){
//				System.out.println(names[i]);	
//			}
			showServerReply(ftpClient);
			
		} catch (FTPConnectionClosedException e) {
			e.printStackTrace();
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (SocketException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
	}
	
	
	public void reloadFileNames() {
		fileNameComboBox.getItems().clear();
		
		// if the directory does not exist, create it
		if (!pathsFolder.exists()) {
		    System.out.println("creating directory: " + pathsFolder.getName());
		    boolean result = false;

		    try{
		    	pathsFolder.mkdir();
		        result = true;
		    } 
		    catch(SecurityException se){
		        //handle it
		    }        
		    if(result) {    
		        System.out.println("DIR created");  
		    }
		}
		
		pathsInFolder = pathsFolder.listFiles();
		
		Arrays.sort(pathsInFolder);
		
		for(int i = 0; i < pathsInFolder.length; i++){
			if(pathsInFolder[i].isFile() && !pathsInFolder[i].getName().equals(".DS_Store")){
//				System.out.println(pathsInFolder[i].getName());
				fileNameComboBox.getItems().add(pathsInFolder[i].getName());
			}
		}
	}
	
	public void reformatPathOptions() {
		if(stageHeight < 600) {
			System.out.println(stageHeight);
			
			GridPane pathOptionsBtns = new GridPane();
			
			pathOptions.getChildren().remove(addPointBtn);
			pathOptions.getChildren().remove(submitBtn);
			pathOptions.getChildren().remove(horizontalMirrorPathBtn);
			pathOptions.getChildren().remove(verticalMirrorPathBtn);
			
			pathOptionsBtns.add(addPointBtn, 0, 0);
			pathOptionsBtns.add(submitBtn, 1, 0);
			pathOptionsBtns.add(horizontalMirrorPathBtn, 2, 0);
			pathOptionsBtns.add(verticalMirrorPathBtn, 3, 0);
			
			pathOptions.add(pathOptionsBtns, 0, 0, 2, 1);
			
			horizontalMirrorPathBtn.setMaxWidth(pathOptionsWidth/2);
			verticalMirrorPathBtn.setMaxWidth(pathOptionsWidth/2);
			submitBtn.setMaxWidth(pathOptionsWidth/2);
			addPointBtn.setMaxWidth(pathOptionsWidth/2);
			
			addPointBtn.setText("Point");
			horizontalMirrorPathBtn.setText("H Mirror");
			verticalMirrorPathBtn.setText("V Mirror");
		} else {
//			System.out.println("Stage Height is " + stageHeight);
		}
	}
	
	public void setPathsFolder() {	//	Not yet function. For future use.
		
		System.out.println(System.getProperty("user.home"));
		
		String newPathsFolderString = System.getProperty("user.home") + "/Desktop/Autonomous-Paths2/";
		
	    pathsFolderString = newPathsFolderString;
		pathsFolder = new File(pathsFolderString); 	// Default Paths Folder
		pathsInFolder = pathsFolder.listFiles();
		
	}
	
}
