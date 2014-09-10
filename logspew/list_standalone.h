#ifndef __LIST_h
#define __LIST_h

/**
 * \file
 * ListNode<T> and LinkedList<T> classes which handle a simple doubly-linked,
 * memory-efficient list. The nodes
 * can be in multiple lists, which is supported by each node containing
 * an array of list nodes (ListNode<T>).
 * \code
 * class SomeElement {
 * public:
 *     ListNode<SomeElement> listnodes[2]; // can be in two lists
 *     ...
 * };
 * 
 * class ThingThatUsesLists {
 *     LinkedList<SomeElement,0> listOne; // one of the lists
 *     LinkedList<SomeElement,1> listTwo; // another list
 * ..
 * };
 * \endcode
 */

/// This is a list node, which needs to be a field in any object
/// which is to be part of a LinkedList. In fact, there
/// needs to be an array of these, because each object may be in
/// more than one list. LinkedList needs to know the index
/// of the node for the list you're using.
///
/// To make an object of class MyClass capable of being part of a list, therefore,
/// add the line
///
///    ListNode<MyClass> listnodes[N];
///
/// to MyClass. Annoyingly, this has to be public; there seems to be no way
/// to template a friend.

template <class T> struct ListNode
{
public:
    /// return the next item in the list from this one
    T *next() { return nxt; }
    
    /// return the previous item in the list from this one
    
    T *prev() { return prv; }
    
    // sadly these have to be public
    
    T *nxt;	//!< next item in list
    T *prv;	//!< previous item in list
};

/// -linked list, memory allocated from the arena passed in to the Init() method.
/// Items in the list must contain a ListNode array (see ListNode)

template <class T, int idx> class LinkedList
{
    friend class ListNode< T>;
public:
    
    LinkedList(){
        reset();
    }
    
    /// empty the list
    
    void reset()
    {
        ct = 0;
        hd = tl = NULL;
    }
    
    
    /// return the number of entries in the list
    
    int entries() { return ct; }
    
    /// add an item to the start of the list
    void addToHead(T *o)
    {
        ct++;
        ListNode<T> *p = o->listnodes+idx;
        if(hd)
        {
            hd->listnodes[idx].prv= o;
            p->prv = NULL;
            p->nxt = hd;
            hd = o;
        }
        else
        {
            hd = tl = o;
            p->nxt = p->prv = NULL;
        }
    }
    
    /// add an item to the end of the list
    void addToTail(T *o)
    {
        ct++;
        ListNode<T> *p = o->listnodes+idx;
        if(hd)
        {
            tl->listnodes[idx].nxt = o;
            p->nxt = NULL;
            p->prv = tl;
            tl = o;
        }
        else
        {
            hd = tl = o;
            p->nxt = p->prv = NULL;
        }
    }
    
    /// remove an item from the list
    void remove(T *o)
    {
        ListNode<T> *p = o->listnodes+idx;
        ct--;
        if(p->prv)
            p->prv->listnodes[idx].nxt = p->nxt;
        else
            hd = p->nxt;
        if(p->nxt)
            p->nxt->listnodes[idx].prv = p->prv;
        else
            tl = p->prv;
    }
    
    
    /// return the head of the list
    T *head() { return hd; }
    
    /// return the tail of the list
    T *tail() { return tl; }
    
    /// does the list contain the item?
    bool contains(T *o)
    {
        T *i;
        
        for(i=hd;i;i=i->listnodes[idx].nxt)
            if(i==o)return true;
        return false;
    }
    
    /// return the nth item in the list, or NULL
    T *getItemByIndex(int n)
    {
        T *i;
        int index=0;
        
        for(i=hd;i;i=i->listnodes[idx].nxt)
        {
            if(index==n)return i;
            index++;
        }
        return NULL;
    }
    
    /// get the index of an item or -1
    int getIndexOf(T *item)
    {
        T *i;
        int index=0;
        
        for(i=hd;i;i=i->listnodes[idx].nxt)
        {
            if(item==i)return index;
            index++;
        }
        return -1;
    }
    
    /// shallow-copy a list from another
    void copy(LinkedList<T,idx> &list)
    {
        hd = list.hd;
        tl = list.tl;
        ct = list.ct;
    }
    
    /// join a list onto the end of another
    void join(LinkedList<T,idx> &list)
    {
        if(tl)
        {
            list.hd->listnodes[idx].prv = tl;
            tl->listnodes[idx].nxt = list.hd;
            tl = list.tl;
        }
        else
        {
            hd = list.hd;
            tl = list.tl;
        }
    }
    
    
    T *next(T *p) {
        return p->listnodes[idx].nxt;
    }
    
    T *prev(T *p) {
        return p->listnodes[idx].prv;
    }
    
    
protected:
    T *hd;	//!< the head of the list
    T *tl; //!< the tail of the list
    int ct;	//!< the number of entries in the list
};


#endif /* __LIST_h */
