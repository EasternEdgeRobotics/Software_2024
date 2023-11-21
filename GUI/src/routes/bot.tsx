import { AlertCircle, CheckCircle2 } from "lucide-react";
import {
  Grid,
  Paper,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
} from "@mui/material";
import { useRos } from "../components/ROS";

function StatusIndicator(props: { statement: any }) {
  if (props.statement) return <CheckCircle2 color="lime" />;
  return <AlertCircle color="red" />;
}

export function Component() {
  const { ros } = useRos();
  const status = [{ name: "ROS", status: ros.isConnected }];

  return (
    <Grid container spacing={2}>
      <Grid item xs={1 / 9} />
      <Grid item xs /> {/* add more stuff here when its actually made */}
      <Grid item xs={3 / 2}>
        <TableContainer component={Paper}>
          <Table>
            <TableHead>
              <TableRow>
                <TableCell align="center">Service</TableCell>
                <TableCell align="center">Status</TableCell>
              </TableRow>
            </TableHead>
            <TableBody>
              {status.map((data) => {
                return (
                  <TableRow
                    key={data.name}
                    sx={{ "&:last-child td, &:last-child th": { border: 0 } }}
                  >
                    <TableCell align="center">{data.name}</TableCell>
                    <TableCell align="center">
                      <StatusIndicator statement={data.status} />
                    </TableCell>
                  </TableRow>
                );
              })}
            </TableBody>
          </Table>
        </TableContainer>
      </Grid>
      <Grid item xs={1 / 9} />
    </Grid>
  );
}
