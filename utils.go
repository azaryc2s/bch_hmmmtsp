package mtsp

import (
	"fmt"
	"math"
	"regexp"
)



func GetNodeIndex(i, j, N, start int) int {
	return start + i*N + j
}

func GetEdgeIndex(i, j, k, N, start int, model string) int {
	if model == MASTERMODEL_ATSP{
		val := start + (i * N * (N-1)) + j * (N-1) + k
		if k > j {
			val--
		}
		return val
	}
	if k < j {
		j, k = k, j
	}
	count := 0
	for l := 0; l < j; l++ {
		count += N - 1 - l
	}
	count += k - j - 1
	return start + (i*((N*N - N)/2)) + count
}

func CalcEdgeDist(coordinates [][]float64, distType string) [][]int {
	n := len(coordinates)
	result := make([][]int, n)
	for node := 0; node < n; node++ {
		result[node] = make([]int, n)
		for node2 := 0; node2 < node; node2++ {
			xDist := coordinates[node][0] - coordinates[node2][0]
			yDist := coordinates[node][1] - coordinates[node2][1]
			var distance int
			if distType == "EUC_2D" {
				distance = int(math.Sqrt(math.Pow(xDist, 2)+math.Pow(yDist, 2)) + 0.5)
			} else if distType == "CEIL_2D" {
				distance = int(math.Ceil(math.Sqrt(math.Pow(xDist, 2) + math.Pow(yDist, 2))))
			}
			result[node][node2] = distance
			result[node2][node] = distance
		}
	}
	return result
}


func Print2DArray(a [][]int) string {
	res := ""
	for _, x := range a {
		for _, y := range x {
			res += fmt.Sprintf("%d,", y)
		}
		res += fmt.Sprintln("")
	}
	return res
}

func SanitizeJsonArrayLineBreaks(json string) string {
	res := fmt.Sprintf("%s", json)
	var numbers = regexp.MustCompile(`\s*([0-9]+),\s+([0-9]+)(,)?`)
	var brackets = regexp.MustCompile(`\[(([0-9]+,)+[0-9]+)\s+\](,?)(\s+)`)
	for numbers.MatchString(res) {
		res = numbers.ReplaceAllString(res, "$1,$2$3")
	}
	for brackets.MatchString(res) {
		res = brackets.ReplaceAllString(res, "[$1]$3$4")
	}
	return res
}
