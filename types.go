package mtsp

import "git.solver4all.com/azaryc2s/gorobi/gurobi"

const (
	Y_BOUNDS_CONT    = "CONT"
	Y_BOUNDS_BIN     = "BIN"
	MASTERMODEL_ATSP = "ATSP"
	MASTERMODEL_TSP  = "TSP"
	STRAT_BCH        = "BCH"
	STRAT_LP         = "LP"
	LBSTRAT_TSP      = "TSP"
	SUBTOURINEQ_TSP  = "TSP"
	SUBTOURINEQ_MTZ  = "MTZ"
	CUT_SEC          = "SEC"
	CUT_BEND_V1      = "BEND_V1"
	CUT_BEND_V2      = "BEND_V2"
	CUT_BEND_V3      = "BEND_V3"
	CUT_BEND_V4      = "BEND_V4"
	CUT_BEND_V5      = "BEND_V5"
	CUT_BEND_V6      = "BEND_V6"
)

type TSPInstance struct {
	Name    string `json:"name"`
	Comment string `json:"comment"`
	Type    string `json:"type"`

	NodeCount       int         `json:"node_count"`
	Dimension       int         `json:"dimension"`
	DisplayDataType string      `json:"display_data_type"`
	EdgeWeightType  string      `json:"edge_weight_type"`
	NodeCoordinates [][]float64 `json:"node_coordinates"`
	EdgeWeights     [][]int     `json:"edge_weights"`
}

type MTSPInstance struct {
	Name    string `json:"name"`
	Comment string `json:"comment"`
	Type    string `json:"type"`

	NodeCount       int         `json:"node_count"`
	DisplayDataType string      `json:"display_data_type"`
	EdgeWeightType  string      `json:"edge_weight_type"`
	NodeCoordinates [][]float64 `json:"node_coordinates"`
	EdgeWeights     [][]int     `json:"edge_weights"`

	ps [][]int
	pp []int

	Depots       []int `json:"depots"`
	VehicleCount int   `json:"vehicle_count"`
	TravelSpeeds []int `json:"travel_speeds"`

	Solution *MTSPSolution
}

type MTSPSolution struct {
	Obj        int     `json:"obj"`
	LBound     int     `json:"lbound"`
	UBound     int     `json:"ubound"`
	Optimal    bool    `json:"optimal"`
	RouteCosts []int   `json:"route_costs"`
	Routes     [][]int `json:"routes"`
	TSPLength  int     `json:"tsp_length"`

	Time    string  `json:"time"`
	System  SysInfo `json:"system"`
	Comment string  `json:"comment"`
}

// SysInfo saves the basic system information
type SysInfo struct {
	Platform string
	CPU      string
	RAM      string
}

type MTSPModel struct {
	GModel       *gurobi.Model
	GEnv         *gurobi.Env
	GCuts        ArrayStringFlags
	GMastermodel string
	EdgeWeights  [][]int
	TravelSpeeds []int
	ps           [][]int
	pp           []int
	BestSol      MTSPSolution
	NewBestSol   bool
	N            int
	M            int
	VarNames     []string
	CMax         int
	XStart       int
	YStart       int
	XCount       int
	YCount       int
	VarCount     int
}
