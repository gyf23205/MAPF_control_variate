# CPPaDO

C++ Implementation of Combinatorial Path Planning Among Dynamic Obstacles

The code is distributed for academic and non-commercial use. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


## Usage

### Compile and toy example

- (cd to the folder of this repo cpp-do/)

- mkdir build

- cd build

- cmake ..

- make

- ./api <map_file> <input_file> <output_file>


### Notes

- <map_file>: path of the map file, e.g. ./maps/random10/random512-10-0.map
- <input_file>: path of input file, which defines:
-  source
-  target set
-  node constraints in the graph
-  <output_file>: path of file to store results, e.g., ./output.txt
-  Example: ./api ../maps/arena/arena.map ../test/test.json ../test/output.txt


### Input file format description

- Below is the description of the JSON structure:
```json
{
  "data": [
    {
      "source": <source-node-id>,
      "targetSet": [<target-node-id-1>, <target-node-id-2>, ...],
      "node_constraints": {
        "<node-id-1>": [<constraint-1>, <constraint-2>, ...],
        "<node-id-2>": [<constraint-1>, <constraint-2>, ...],
        ...
      }
    }
  ]
}
```
e.g.
```json
{
  "data": [
    {
      "source": 1293,
      "targetSet": [1907, 2108, 2234, 1123, 1326],
      "node_constraints": {
        "2108": [0, 10, 20],
        "1907": [5, 15],
        "2234": [10, 20],
        "1326": [0, 12],
        "6": [3],
        "7": [7, 22],
        "8": [15, 25]
      }
    }
  ]
}
```

### Output file format description

- General Result: shows the runtime information and nodes included in the path with corresponding time.
- Detailed Actions: shows the action with certain location with corresponding time.
  e.g.
-  1 1 0 WAIT     // start at (1, 1) at time 0, wait
-  1 1 10 MOVE    // wait for 10s, then move
-  2 1 11 SERVE   // reach (2, 1), then execute task
-  2 1 12 MOVE    // finish task, then move
-  2 2 13 SERVE   // reach (2, 2), then execute task
-  2 2 15 FINISH  // finish task, all done
  
  
## References

[1] Ren, Zhongqiang, Sivakumar Rathinam, Maxim Likhachev, and Howie Choset. "Multi-Objective Safe-Interval Path Planning With Dynamic Obstacles." IEEE Robotics and Automation Letters 7, no. 3 (2022): 8154-8161.\
[[Bibtex](https://wonderren.github.io/files/bibtex_ren22mosipp.txt)]
[[Paper](https://github.com/wonderren/wonderren.github.io/blob/master/files/ren22_mosipp_RAL_IROS22.pdf)]
