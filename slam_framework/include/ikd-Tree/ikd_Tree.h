#pragma once
#include <stdio.h>
#include <queue>
#include <pthread.h>
#include <chrono>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <algorithm>
#include <memory.h>
#include <pcl/point_types.h>
#include "tictoc.h"
#define EPSS 1e-6
#define Minimal_Unbalanced_Tree_Size 10
#define Multi_Thread_Rebuild_Point_Num 1500
#define DOWNSAMPLE_SWITCH true
#define ForceRebuildPercentage 0.2
#define Q_LEN 1000000

using namespace std;

// typedef pcl::PointXYZINormal PointType_ikd;
// typedef vector<PointType_ikd, Eigen::aligned_allocator<PointType_ikd>>  PointVector_ikd;

struct BoxPointType
{
    float vertex_min[3];
    float vertex_max[3];
};

enum operation_set
{
    ADD_POINT,
    DELETE_POINT,
    DELETE_BOX,
    ADD_BOX,
    DOWNSAMPLE_DELETE,
    PUSH_DOWN
};

enum delete_point_storage_set
{
    NOT_RECORD,
    DELETE_POINTS_REC,
    MULTI_THREAD_REC
};

template <typename PointType_ikd>
class KD_TREE
{
    // using MANUAL_Q_ = MANUAL_Q<typename PointType_ikd>;
    // using PointVector_ikd = std::vector<PointType_ikd>;
    
    // using MANUAL_Q_ = MANUAL_Q<typename PointType_ikd>;
public:
    using PointVector_ikd = std::vector<PointType_ikd, Eigen::aligned_allocator<PointType_ikd>>;
    using Ptr = std::shared_ptr<KD_TREE<PointType_ikd>>;
    
    struct KD_TREE_NODE
    {
        PointType_ikd point;
        int division_axis;
        int TreeSize = 1;
        int invalid_point_num = 0;
        int down_del_num = 0;
        bool point_deleted = false;
        bool tree_deleted = false;
        bool point_downsample_deleted = false;
        bool tree_downsample_deleted = false;
        bool need_push_down_to_left = false;
        bool need_push_down_to_right = false;
        bool working_flag = false;
        pthread_mutex_t push_down_mutex_lock;
        float node_range_x[2], node_range_y[2], node_range_z[2];
        float radius_sq;
        KD_TREE_NODE *left_son_ptr = nullptr;
        KD_TREE_NODE *right_son_ptr = nullptr;
        KD_TREE_NODE *father_ptr = nullptr;
        // For paper data record
        float alpha_del;
        float alpha_bal;
    };

    struct Operation_Logger_Type
    {
        PointType_ikd point;
        BoxPointType boxpoint;
        bool tree_deleted, tree_downsample_deleted;
        operation_set op;
    };
    // static const PointType_ikd zeroP;

    struct PointType_CMP
    {
        PointType_ikd point;
        float dist = 0.0;
        PointType_CMP(PointType_ikd p = PointType_ikd(), float d = INFINITY)
        {
            this->point = p;
            this->dist = d;
        };
        bool operator<(const PointType_CMP &a) const
        {
            if (fabs(dist - a.dist) < 1e-10)
                return point.x < a.point.x;
            else
                return dist < a.dist;
        }
    };

    class MANUAL_HEAP
    {

    public:
        MANUAL_HEAP(int max_capacity = 100)

        {
            cap = max_capacity;
            heap = new PointType_CMP[max_capacity];
            heap_size = 0;
        }

        ~MANUAL_HEAP()
        {
            delete[] heap;
        }
        void pop()
        {
            if (heap_size == 0)
                return;
            heap[0] = heap[heap_size - 1];
            heap_size--;
            MoveDown(0);
            return;
        }
        PointType_CMP top()
        {
            return heap[0];
        }
        void push(PointType_CMP point)
        {
            if (heap_size >= cap)
                return;
            heap[heap_size] = point;
            FloatUp(heap_size);
            heap_size++;
            return;
        }
        int size()
        {
            return heap_size;
        }
        void clear()
        {
            heap_size = 0;
            return;
        }

    private:
        PointType_CMP *heap;
        void MoveDown(int heap_index)
        {
            int l = heap_index * 2 + 1;
            PointType_CMP tmp = heap[heap_index];
            while (l < heap_size)
            {
                if (l + 1 < heap_size && heap[l] < heap[l + 1])
                    l++;
                if (tmp < heap[l])
                {
                    heap[heap_index] = heap[l];
                    heap_index = l;
                    l = heap_index * 2 + 1;
                }
                else
                    break;
            }
            heap[heap_index] = tmp;
            return;
        }
        void FloatUp(int heap_index)
        {
            int ancestor = (heap_index - 1) / 2;
            PointType_CMP tmp = heap[heap_index];
            while (heap_index > 0)
            {
                if (heap[ancestor] < tmp)
                {
                    heap[heap_index] = heap[ancestor];
                    heap_index = ancestor;
                    ancestor = (heap_index - 1) / 2;
                }
                else
                    break;
            }
            heap[heap_index] = tmp;
            return;
        }
        int heap_size = 0;
        int cap = 0;
    };

    class MANUAL_Q
    {
    private:
        int head = 0, tail = 0, counter = 0;
        Operation_Logger_Type q[Q_LEN];
        bool is_empty;

    public:
        void pop()
        {
            if (counter == 0)
                return;
            head++;
            head %= Q_LEN;
            counter--;
            if (counter == 0)
                is_empty = true;
            return;
        }
        Operation_Logger_Type front()
        {
            return q[head];
        }
        Operation_Logger_Type back()
        {
            return q[tail];
        }
        void clear()
        {
            head = 0;
            tail = 0;
            counter = 0;
            is_empty = true;
            return;
        }
        void push(Operation_Logger_Type op)
        {
            q[tail] = op;
            counter++;
            if (is_empty)
                is_empty = false;
            tail++;
            tail %= Q_LEN;
        }
        bool empty()
        {
            return is_empty;
        }
        int size()
        {
            return counter;
        }
    };

private:
    // Multi-thread Tree Rebuild
    bool termination_flag = false;
    bool rebuild_flag = false;
    pthread_t rebuild_thread;
    pthread_mutex_t termination_flag_mutex_lock, rebuild_ptr_mutex_lock, working_flag_mutex, search_flag_mutex;
    pthread_mutex_t rebuild_logger_mutex_lock, points_deleted_rebuild_mutex_lock;
    // queue<Operation_Logger_Type> Rebuild_Logger;
    MANUAL_Q Rebuild_Logger;
    PointVector_ikd Rebuild_PCL_Storage;
    KD_TREE_NODE **Rebuild_Ptr = nullptr;
    int search_mutex_counter = 0;
    static void *multi_thread_ptr(void *arg);
    void multi_thread_rebuild();
    void start_thread();
    void stop_thread();
    void run_operation(KD_TREE_NODE **root, Operation_Logger_Type operation);
    // KD Tree Functions and augmented variables
    int Treesize_tmp = 0, Validnum_tmp = 0;
    float alpha_bal_tmp = 0.5, alpha_del_tmp = 0.0;
    float delete_criterion_param = 0.5f;
    float balance_criterion_param = 0.7f;
    float downsample_size = 0.2f;
    bool Delete_Storage_Disabled = false;
    KD_TREE_NODE *STATIC_ROOT_NODE = nullptr;
    PointVector_ikd Points_deleted;
    PointVector_ikd Downsample_Storage;
    PointVector_ikd Multithread_Points_deleted;
    void InitTreeNode(KD_TREE_NODE *root);
    void Test_Lock_States(KD_TREE_NODE *root);
    void BuildTree(KD_TREE_NODE **root, int l, int r, PointVector_ikd &Storage);
    void Rebuild(KD_TREE_NODE **root);
    int Delete_by_range(KD_TREE_NODE **root, BoxPointType boxpoint, bool allow_rebuild, bool is_downsample);
    void Delete_by_point(KD_TREE_NODE **root, PointType_ikd point, bool allow_rebuild);
    void Add_by_point(KD_TREE_NODE **root, PointType_ikd point, bool allow_rebuild, int father_axis);
    void Add_by_range(KD_TREE_NODE **root, BoxPointType boxpoint, bool allow_rebuild);
    void Search(KD_TREE_NODE *root, int k_nearest, PointType_ikd point, MANUAL_HEAP &q, float max_dist); //priority_queue<PointType_CMP>
    void Search_by_range(KD_TREE_NODE *root, BoxPointType boxpoint, PointVector_ikd &Storage);
    void Search_by_range_non_recur(KD_TREE_NODE *root, BoxPointType boxpoint, PointVector_ikd &Storage);

    void Search_by_radius(KD_TREE_NODE *root, PointType_ikd point, float radius, PointVector_ikd &Storage);
    bool Criterion_Check(KD_TREE_NODE *root);
    void Push_Down(KD_TREE_NODE *root);
    void Update(KD_TREE_NODE *root);
    void delete_tree_nodes(KD_TREE_NODE **root);
    void downsample(KD_TREE_NODE **root);
    bool same_point(PointType_ikd a, PointType_ikd b);
    float calc_dist(PointType_ikd a, PointType_ikd b);
    float calc_box_dist(KD_TREE_NODE *node, PointType_ikd point);
    static bool point_cmp_x(PointType_ikd a, PointType_ikd b);
    static bool point_cmp_y(PointType_ikd a, PointType_ikd b);
    static bool point_cmp_z(PointType_ikd a, PointType_ikd b);

public:
    KD_TREE(float delete_param = 0.5, float balance_param = 0.6, float box_length = 0.2);
    ~KD_TREE();
    void Set_delete_criterion_param(float delete_param)
    {
        delete_criterion_param = delete_param;
    }
    void Set_balance_criterion_param(float balance_param)
    {
        balance_criterion_param = balance_param;
    }
    void set_downsample_param(float downsample_param)
    {
        downsample_size = downsample_param;
    }
    void InitializeKDTree(float delete_param = 0.5, float balance_param = 0.7, float box_length = 0.2);
    int size();
    int validnum();
    void root_alpha(float &alpha_bal, float &alpha_del);
    void Build(PointVector_ikd point_cloud);
    void Nearest_Search(PointType_ikd point, int k_nearest, PointVector_ikd &Nearest_Points, vector<float> &Point_Distance, float max_dist = INFINITY);
    void Box_Search(const BoxPointType &Box_of_Point, PointVector_ikd &Storage);
    void Radius_Search(PointType_ikd point, const float radius, PointVector_ikd &Storage);
    int Add_Points(PointVector_ikd &PointToAdd, bool downsample_on);
    void Add_Point_Boxes(vector<BoxPointType> &BoxPoints);
    void Delete_Points(PointVector_ikd &PointToDel);
    int Delete_Point_Boxes(vector<BoxPointType> &BoxPoints);
    void flatten(KD_TREE_NODE *root, PointVector_ikd &Storage, delete_point_storage_set storage_type);
    void acquire_removed_points(PointVector_ikd &removed_points);
    BoxPointType tree_range();
    PointVector_ikd PCL_Storage;
    KD_TREE_NODE *Root_Node = nullptr;
    int max_queue_size = 0;
};

// template <typename PointType_ikd>
// PointType_ikd KD_TREE<PointType_ikd>::zeroP = PointType_ikd(0,0,0);
