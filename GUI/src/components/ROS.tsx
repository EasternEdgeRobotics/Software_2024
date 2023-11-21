import ROSLIB from "roslib";
import {
  createContext,
  useContext,
  useEffect,
  PropsWithChildren,
  useState,
} from "react";

const RosContext = createContext<ROSLIB.Ros | undefined>(undefined);

type RosProviderProps = {
  rosURL: string;
};

export function RosProvider(props: PropsWithChildren<RosProviderProps>) {
  const [ros] = useState(new ROSLIB.Ros({}));

  useEffect(() => {
    const onConnection = () => {
      console.log("ROS Connected!");
    };

    const onClose = () => {
      console.log("ROS Disconnected!");
    };

    const onError = (error: Error) => {
      console.log(error);
    };

    ros.on("connection", onConnection);
    ros.on("close", onClose);
    ros.on("error", onError);
    ros.connect(props.rosURL);

    return () => {
      ros.off("connection", onConnection);
      ros.off("close", onClose);
      ros.off("error", onError);
      ros.close();
    };
  }, []);

  return (
    <RosContext.Provider value={ros}>{props.children}</RosContext.Provider>
  );
}

export function useRos() {
  const ros = useContext(RosContext);

  if (ros === undefined) {
    throw new Error("useRos must be used within RosProvider");
  }

  return { ros };
}
