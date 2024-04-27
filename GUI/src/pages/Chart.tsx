import React from 'react';
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend,
} from 'chart.js';
import { Line } from 'react-chartjs-2';

ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend
);

interface ControllerAppProps {
  labels: string[];
  data: number[];
  x_label: string;
  y_label: string;
}

export const ControllerApp: React.FC<ControllerAppProps> = ({ labels, data, x_label, y_label }) => {
  const chartData = {
    labels,
    datasets: [
      {
        label: 'Dataset 1',
        data,
        borderColor: 'rgb(255, 99, 132)',
        backgroundColor: 'rgba(255, 99, 132, 0.5)',
      },
    ],
  };

  const options = {
    responsive: true,
    plugins: {
      legend: {
        position: 'top' as const,
      },
      title: {
        display: true,
        text: 'Chart.js Line Chart',
      },
    },
    scales: {
      x: {
        title: {
          display: true,
          text: x_label,
        },
      },
      y: {
        title: {
          display: true,
          text: y_label,
        },
      },
    },
  };

  return <Line options={options} data={chartData} />
};