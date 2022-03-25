package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"git.solver4all.com/azaryc2s/mtsp"
	"io/fs"
	"io/ioutil"
	"log"
	"math"
	"math/rand"
	"regexp"
	"strings"
	"time"
)

var speeds mtsp.ArrayStringFlags
var nodes mtsp.ArrayIntFlags
var vehicles mtsp.ArrayIntFlags
var name *string
var input *string
var output *string
var count *int
var rngStart *int
var rngEnd *int
var vehGroupSize *int
var xTo *int
var yTo *int
var w *string

func main() {
	flag.Var(&speeds, "s", "List of speed-generation strategies. (ONE|RNG|RNG-GROUP)")
	flag.Var(&nodes, "n", "List of number of nodes")
	flag.Var(&vehicles, "m", "List of number of vehicles")
	name = flag.String("name", "zarychta", "Name for the instance")
	//comment := flag.String("comment", "Zarychta generated hmmVRP-Instance", "Comment for the instances")
	input = flag.String("inputDir", "", "Input directory with files as base problem (to extract coordinates from)")
	output = flag.String("outputDir", ".", "Output directory")
	count = flag.Int("count", 1, "Number of instances per combination")
	rngStart = flag.Int("rngStart", 1, "The lowest value for vehicle speed")
	rngEnd = flag.Int("rngEnd", 10, "The highest added value for vehicle speed(actual max value is start+end-1)")
	vehGroupSize = flag.Int("vehGroupSize", 3, "The size of the vehicle groups, when using rng-group speed strategy")
	xTo = flag.Int("x", 10000, "Max value on the x-axis")
	yTo = flag.Int("y", 10000, "Max value on the y-axis")
	w = flag.String("w", "EUC_2D", "EDGE_WEIGHT_TYPE - how the distance between nodes is calculated.")

	flag.Parse()
	var fileInst []mtsp.TSPInstance
	var genFromFiles bool
	if *input != "" {
		genFromFiles = true
		var dir []fs.FileInfo
		var err error

		dir, err = ioutil.ReadDir(*input)
		if err != nil {
			log.Printf("Couldn't open directory %s: %s\n", *input, err.Error())
			return
		}

		for _, f := range dir {
			fileName := *input + "/" + f.Name()
			if strings.Contains(fileName, ".json") {
				inst := mtsp.TSPInstance{}
				instStr, err := ioutil.ReadFile(fileName)
				if err != nil {
					log.Printf("Couldn't read %s: %s\n", f.Name(), err.Error())
					return
				}
				err = json.Unmarshal(instStr, &inst)
				if err != nil {
					log.Printf("Couldn't parse %s: %s\n", f.Name(), err.Error())
					return
				}
				fileInst = append(fileInst, inst)
				nodes.Set(fmt.Sprintf("%d", inst.Dimension))
			}
		}
	}

	for l := 0; l < *count; l++ {
		rand.Seed(time.Now().UnixNano())
		for i := 0; i < len(nodes); i++ {
			n := nodes[i]
			var coordinatesArray [][]float64
			if genFromFiles {
				coordinatesArray = fileInst[i].NodeCoordinates
				*name = fileInst[i].Name
			} else {
				coordinatesArray = make([][]float64, n)
				edgeWeights := make([][]int, n)
				for node := 0; node < n; node++ {
					x := rand.Intn(*xTo)
					y := rand.Intn(*yTo)
					coordinatesArray[node] = []float64{float64(x), float64(y)}
					edgeWeights[node] = make([]int, n)
					for node2 := 0; node2 < node; node2++ {
						xDist := coordinatesArray[node][0] - coordinatesArray[node2][0]
						yDist := coordinatesArray[node][1] - coordinatesArray[node2][1]
						var distance int
						if *w == "EUC_2D" {
							distance = int(math.Sqrt(math.Pow(xDist, 2)+math.Pow(yDist, 2)) + 0.5)
						} else if *w == "CEIL_2D" {
							distance = int(math.Ceil(math.Sqrt(math.Pow(xDist, 2) + math.Pow(yDist, 2))))
						}
						edgeWeights[node][node2] = distance
						edgeWeights[node2][node] = distance
					}
				}
			}
			depots := []int{0}
			for j := 0; j < len(vehicles); j++ {
				m := vehicles[j]
				for k := 0; k < len(speeds); k++ {
					s := speeds[k]
					speedsArray := make([]int, m)
					if s == "ONE" {
						for pr := 0; pr < m; pr++ {
							speedsArray[pr] = 1
						}
					} else if s == "RNG" {
						for pr := 0; pr < m; pr++ {
							speedsArray[pr] = *rngStart + rand.Intn(*rngEnd)
						}
					} else if s == "RNG-GROUP" {
						g := int(math.Max(float64(m / *vehGroupSize), 1.0))
						r := 0
						for pr := 0; pr < m; pr++ {
							if pr % g == 0{
								r = *rngStart + rand.Intn(*rngEnd)
							}
							speedsArray[pr] = r
						}
					}

					comment := fmt.Sprintf("%s instance Nr. %d with %d nodes, %d vehicles and speeds generated as %s", *name, l, n, m, s)
					instName := fmt.Sprintf("%s_%d_%d_%s_%d", *name, n, m, s, l)
					hmmVRPInstance := mtsp.MTSPInstance{Name: instName, Comment: comment, Type: "hmmVRP", NodeCount: n, VehicleCount: m, TravelSpeeds: speedsArray, NodeCoordinates: coordinatesArray, Depots: depots, DisplayDataType: "COORD_DISPLAY", EdgeWeightType: *w}

					jsonInst, err := json.MarshalIndent(hmmVRPInstance, "", "\t")
					if err != nil {
						log.Fatal(err)
					}

					jsonInst = []byte(sanitizeJsonArrayLineBreaks(string(jsonInst)))
					err = ioutil.WriteFile(fmt.Sprintf("%s/%s.json", *output, instName), jsonInst, 0644)
					if err != nil {
						log.Fatal(err)
					}
				}
			}
		}
	}
}

func sanitizeJsonArrayLineBreaks(json string) string {
	res := fmt.Sprintf("%s", json)
	var numbers = regexp.MustCompile(`\s*([-]?[0-9]+(\.[0-9]+)?),\s+([-]?[0-9]+(\.[0-9]+)?)(,)?`)
	var brackets = regexp.MustCompile(`\[(([-]?[0-9]+(\.[0-9]+)?,)+[-]?[0-9]+(\.[0-9]+)?)\s+\](,?)(\s+)`)
	for numbers.MatchString(res) {
		res = numbers.ReplaceAllString(res, "$1,$3$5")
	}
	for brackets.MatchString(res) {
		res = brackets.ReplaceAllString(res, "[$1]$5$6")
	}
	return res
}
