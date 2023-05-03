#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#define APP_MAX_NODES 40

#define ANCHOR0 0.65	//kotva 1 1 metr
#define ANCHOR1 0.5		//kotva 2 2 metry

#define ANCHOR0_address	0x00	//hexa address of anchor node 0
#define ANCHOR1_address	0x01	//hexa address of anchor node 1

struct Node
{
	int		id;
	int		srcAddr;
	int		rssi;
};


struct DistanceTableRow_t
{
	int		node_ID;
	int		node_address;
	int		average_RSSI;	//average_RSSI in dBm
	float	distance;
	int		niRecalculate;
	int		measured_RSSI_count; //measured_RSSI_count
	int		measured_RSSI[5];
};

int distanceTableRowCount = 0;
int rssi_counter = 0;
double eta = 0;
bool niSet = false;
struct DistanceTableRow_t distanceTable[APP_MAX_NODES];

void appRSSIInd(struct Node node);

void main() {

	struct Node row1 = { 1, 0, -60 };

	struct Node row2 = { 1, 0, -63 };
	struct Node row3 = { 2, 1, -55 };
	
	struct Node row4 = { 1, 0, -60 };
	struct Node row5 = { 2, 1, -61 };
	struct Node row6 = { 3, 2, -62 };
	
	struct Node row7 = { 1, 0, -60 };
	struct Node row8 = { 1, 0, -61 };
	struct Node row9 = { 3, 2, -62 };

	struct Node row10 = { 3, 2, -5 };

	//struct Node nodes[4] = { row1, row2, row3, row4 };
	printf("############# 1\r\n");
	appRSSIInd(row1);
	printf("############# 2\r\n");
	appRSSIInd(row2);
	printf("############# 3\r\n");
	appRSSIInd(row3);
	printf("############# 4\r\n");
	appRSSIInd(row4);
	printf("############# 5\r\n");
	appRSSIInd(row5);
	printf("############# 6\r\n");
	appRSSIInd(row6);
	printf("############# 7\r\n");
	appRSSIInd(row7);
	printf("############# 8\r\n");
	appRSSIInd(row8);
	printf("############# 9\r\n");
	appRSSIInd(row9);
	printf("############# 10\r\n");
	appRSSIInd(row10);

	/*printf("Node ID: %-12d\n", distanceTable[0].node_ID);
	printf("Node address: %.0lf\n", distanceTable[0].node_address);
	printf("Node address: %.2f\n", distanceTable[0].average_RSSI);

	printf("Node ID: %-12d\n", distanceTable[1].node_ID);
	printf("Node address: %.0lf\n", distanceTable[1].node_address);
	printf("Node address: %.2f\n", distanceTable[1].average_RSSI);

	printf("Node ID: %-12d\n", distanceTable[2].node_ID);
	printf("Node address: %.0lf\n", distanceTable[2].node_address);
	printf("Node address: %.2f\n", distanceTable[2].average_RSSI);*/
}

void appRSSIInd(struct Node node) {
	int nodePresentInTable = 0;
	printf("RSSI message from 0x%X, id %d, RSSI: %d \r\n", node.srcAddr, node.id, node.rssi);

	//############ TABLE FOR DISTANCE
	for (int i = 0; i < distanceTableRowCount; i++)
	{
		if (node.srcAddr == distanceTable[i].node_address)
		{
			printf("Rows in table: %d\r\n", distanceTableRowCount);
			printf("Updating node: %d\r\n", distanceTable[i].node_address);
			double tempDistance = 0;

			if (distanceTable[i].measured_RSSI_count == 5)
			{
				distanceTable[i].measured_RSSI_count = 0;
			}
			distanceTable[i].measured_RSSI[distanceTable[i].measured_RSSI_count] = node.rssi;
			nodePresentInTable = 1;
			//printf("Measured distance count: %d\r\n", distanceTable[i].measured_RSSI_count);
			distanceTable[i].measured_RSSI_count += 1;
			//printf("niRecalculate value: %d\r\n", distanceTable[i].niRecalculate);
			distanceTable[i].niRecalculate += 1;
			for (int p = 0; p < 5; p++)
			{
				if (p == 0) {
					printf("| %d |", distanceTable[i].measured_RSSI[p]);
				} else {
					printf(" %d |", distanceTable[i].measured_RSSI[p]);
				}
			}
			printf("\r\n");
			if ((distanceTable[i].niRecalculate >= 5 && (distanceTableRowCount >= 2) && (node.srcAddr == ANCHOR0_address)))
			{
				niSet = true;
				distanceTable[i].niRecalculate = 0;
				eta = (double)(distanceTable[0].average_RSSI - distanceTable[1].average_RSSI) / (10 * log10((double)ANCHOR1 / (double)ANCHOR0));
				printf("eta: %lf\r\n", eta);
			}

			for (int j = 0; j < 5; j++)
			{
				tempDistance += distanceTable[i].measured_RSSI[j];
				//printf("Measured i at index %d: %d\r\n", j, distanceTable[i].measured_RSSI[j]);
			}
			distanceTable[i].average_RSSI = tempDistance / 5;
			printf("Average RSSI of node %X is: %d\r\n", distanceTable[i].node_address, distanceTable[i].average_RSSI);
			if (niSet && (node.srcAddr != ANCHOR0_address) && (node.srcAddr != ANCHOR1_address)) {
				distanceTable[i].distance = (double)ANCHOR0 * pow(10, ((distanceTable[0].average_RSSI - distanceTable[i].average_RSSI) / (10 * eta)));
				printf("Distance to node %X is: %f\r\n", distanceTable[i].node_address, distanceTable[i].distance);
			}
			printf("######################################");
			break;
		}
	}

	if (!nodePresentInTable || (distanceTableRowCount == 0))
	{
		double tempDistance = 0;
		distanceTable[distanceTableRowCount].node_ID = node.id;
		distanceTable[distanceTableRowCount].node_address = node.srcAddr;
		printf("New node: 0x%X\r\n", distanceTable[distanceTableRowCount].node_address);
		distanceTable[distanceTableRowCount].measured_RSSI_count = 0;
		distanceTable[distanceTableRowCount].measured_RSSI[distanceTable[distanceTableRowCount].measured_RSSI_count] = node.rssi;
		distanceTable[distanceTableRowCount].measured_RSSI_count += 1;
		distanceTable[distanceTableRowCount].niRecalculate = 1;
		for (int i = 0; i < 5; i++)
		{
			tempDistance += distanceTable[distanceTableRowCount].measured_RSSI[i];
		}
		distanceTable[distanceTableRowCount].average_RSSI = tempDistance / 5;
		distanceTableRowCount += 1;
		printf("Rows in table: %d\r\n", distanceTableRowCount);
	}
	//############ END OF TABLE FOR DISTANCE
}
