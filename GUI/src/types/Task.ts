export type Task = {
    name: string;
    completed?: boolean;
    subTasks?: Task[];
    Points?: number;
    };

export type SubTask = {
    name: string;
    completed: boolean;
    subTasks?: SubTask[];
    Points?: number;
};