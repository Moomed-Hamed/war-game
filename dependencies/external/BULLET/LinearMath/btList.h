#ifndef BT_GEN_LIST_H
#define BT_GEN_LIST_H

class btGEN_Link
{
public:
	btGEN_Link() : m_next(0), m_prev(0) {}
	btGEN_Link(btGEN_Link *next, btGEN_Link *prev) : m_next(next), m_prev(prev) {}

	btGEN_Link *getNext() const { return m_next; }
	btGEN_Link *getPrev() const { return m_prev; }

	bool isHead() const { return m_prev == 0; }
	bool isTail() const { return m_next == 0; }

	void insertBefore(btGEN_Link *link)
	{
		m_next = link;
		m_prev = link->m_prev;
		m_next->m_prev = this;
		m_prev->m_next = this;
	}

	void insertAfter(btGEN_Link *link)
	{
		m_next = link->m_next;
		m_prev = link;
		m_next->m_prev = this;
		m_prev->m_next = this;
	}

	void remove()
	{
		m_next->m_prev = m_prev;
		m_prev->m_next = m_next;
	}

private:
	btGEN_Link *m_next;
	btGEN_Link *m_prev;
};

class btGEN_List
{
public:
	btGEN_List() : m_head(&m_tail, 0), m_tail(0, &m_head) {}

	btGEN_Link *getHead() const { return m_head.getNext(); }
	btGEN_Link *getTail() const { return m_tail.getPrev(); }

	void addHead(btGEN_Link *link) { link->insertAfter(&m_head); }
	void addTail(btGEN_Link *link) { link->insertBefore(&m_tail); }

private:
	btGEN_Link m_head;
	btGEN_Link m_tail;
};

#endif  //BT_GEN_LIST_H
