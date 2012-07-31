/*
 * @(#)SprayAndFocusRouter.java
 *
 * Copyright 2010 by University of Pittsburgh, released under GPLv3.
 * 
 */
package routing;

import java.util.*;

import core.*;

/**
 * An implementation of Spray and Focus DTN routing as described in
 * <em>Spray and Focus: Efficient Mobility-Assisted Routing for Heterogeneous
 * and Correlated Mobility</em> by Thrasyvoulos Spyropoulos et al.
 * 
 * @author PJ Dillon, University of Pittsburgh
 */
public class IG_FerryRouter extends ActiveRouter {
	/** SprayAndFocus router's settings name space ({@value} ) */
	public static final String SPRAYANDFOCUS_NS = "IGFerryRouter";
	/** identifier for the initial number of copies setting ({@value} ) */
	public static final String NROF_COPIES_S = "nrofCopies";
	/**
	 * identifier for the difference in timer values needed to forward on a
	 * message copy
	 */
	public static final String TIMER_THRESHOLD_S = "transitivityTimerThreshold";
	/** Message property key for the remaining available copies of a message */
	public static final String MSG_COUNT_PROP = "IGFerry.copies";
	/**
	 * Message property key for summary vector messages exchanged between direct
	 * peers
	 */
	public static final String SUMMARY_XCHG_PROP = "IGFerry.protoXchg";

	protected static final String SUMMARY_XCHG_IDPREFIX = "summary";
	protected static final double defaultTransitivityThreshold = 60.0;
	protected static int protocolMsgIdx = 0;

	protected int initialNrofCopies;
	protected double transitivityTimerThreshold;

	/** Stores information about nodes with which this host has come in contact */
	protected Map<DTNHost, EncounterInfo> recentEncounters;
	protected Map<DTNHost, Map<DTNHost, EncounterInfo>> neighborEncounters;

	public IG_FerryRouter(Settings s) {
		super(s);
		Settings snf = new Settings(SPRAYANDFOCUS_NS);
		initialNrofCopies = snf.getInt(NROF_COPIES_S);

		if (snf.contains(TIMER_THRESHOLD_S))
			transitivityTimerThreshold = snf.getDouble(TIMER_THRESHOLD_S);
		else
			transitivityTimerThreshold = defaultTransitivityThreshold;

		recentEncounters = new HashMap<DTNHost, EncounterInfo>();
		neighborEncounters = new HashMap<DTNHost, Map<DTNHost, EncounterInfo>>();
	}

	/**
	 * Copy Constructor.
	 * 
	 * @param r
	 *            The router from which settings should be copied
	 */
	public IG_FerryRouter(IG_FerryRouter r) {
		super(r);
		this.initialNrofCopies = r.initialNrofCopies;

		recentEncounters = new HashMap<DTNHost, EncounterInfo>();
		neighborEncounters = new HashMap<DTNHost, Map<DTNHost, EncounterInfo>>();
	}

	@Override
	public MessageRouter replicate() {
		return new IG_FerryRouter(this);
	}

	/**
	 * Called whenever a connection goes up or comes down.
	 */
	@Override
	public void changedConnection(Connection con) {
		super.changedConnection(con);

		/*
		 * The paper for this router describes Message summary vectors (from the
		 * original Epidemic paper), which are exchanged between hosts when a
		 * connection is established. This functionality is already handled by
		 * the simulator in the protocol implemented in startTransfer() and
		 * receiveMessage().
		 * 
		 * Below we need to implement sending the corresponding message.
		 */
		DTNHost thisHost = getHost();
		DTNHost peer = con.getOtherNode(thisHost);

		// do this when con is up and goes down (might have been up for awhile)
		if (recentEncounters.containsKey(peer)) {
			EncounterInfo info = recentEncounters.get(peer);
			info.updateEncounterTime(SimClock.getTime());
		} else {
			recentEncounters.put(peer, new EncounterInfo(SimClock.getTime()));
		}

		if (!con.isUp()) {
			neighborEncounters.remove(peer);
			return;
		}

		/*
		 * For this simulator, we just need a way to give the other node in this
		 * connection access to the peers we recently encountered; so we
		 * duplicate the recentEncounters Map and attach it to a message.
		 */
		int msgSize = recentEncounters.size() * 64 + getMessageCollection().size() * 8;
		Message newMsg = new Message(thisHost, peer, SUMMARY_XCHG_IDPREFIX + protocolMsgIdx++, msgSize);
		newMsg.addProperty(SUMMARY_XCHG_PROP, /*
											 * new HashMap<DTNHost,
											 * EncounterInfo>(
											 */recentEncounters);

		createNewMessage(newMsg);
	}

	@Override
	public boolean createNewMessage(Message m) {
		makeRoomForNewMessage(m.getSize());

		m.addProperty(MSG_COUNT_PROP, new Integer(initialNrofCopies));
		addToMessages(m, true);
		return true;
	}

	@Override
	public Message messageTransferred(String id, DTNHost from) {
		Message m = super.messageTransferred(id, from);

		/*
		 * Here we update our last encounter times based on the information sent
		 * from our peer.
		 */
		Map<DTNHost, EncounterInfo> peerEncounters = (Map<DTNHost, EncounterInfo>) m.getProperty(SUMMARY_XCHG_PROP);
		if (isDeliveredMessage(m) && peerEncounters != null) {
			double distTo = getHost().getLocation().distance(from.getLocation());
			double speed = from.getPath() == null ? 0 : from.getPath().getSpeed();

			if (speed == 0.0)
				return m;

			double timediff = distTo / speed;

			/*
			 * We save the peer info for the utility based forwarding decisions,
			 * which are implemented in update()
			 */
			neighborEncounters.put(from, peerEncounters);

			for (Map.Entry<DTNHost, EncounterInfo> entry : peerEncounters.entrySet()) {
				DTNHost h = entry.getKey();
				if (h == getHost())
					continue;

				EncounterInfo peerEncounter = entry.getValue();
				EncounterInfo info = recentEncounters.get(h);

				/*
				 * We set our timestamp for some node, h, with whom our peer has
				 * come in contact if our peer has a newer timestamp beyond some
				 * threshold.
				 * 
				 * The paper describes timers that count up from the time of
				 * contact. We use fixed timestamps here to accomplish the same
				 * effect, but the computations here are consequently a little
				 * different from the paper.
				 */
				if (!recentEncounters.containsKey(h)) {
					info = new EncounterInfo(peerEncounter.getLastSeenTime() - timediff);
					recentEncounters.put(h, info);
					continue;
				}

				if (info.getLastSeenTime() + timediff < peerEncounter.getLastSeenTime()) {
					recentEncounters.get(h).updateEncounterTime(peerEncounter.getLastSeenTime() - timediff);
				}
			}
			return m;
		}

		// Normal message beyond here

		Integer from_nrofCopies = (Integer) m.getProperty(MSG_COUNT_PROP);

		for (Map.Entry<DTNHost, EncounterInfo> entry : peerEncounters.entrySet()) {

			DTNHost h = entry.getKey();
			if (h == getHost())
				continue;

			Integer h_nrofCopies = (Integer) h.getMessageCollections().getMessage(id).getProperty(MSG_COUNT_PROP);
			double def_From = def(m, from);
			double def_Nexthop = def(h.getMessageCollections().getMessage(id), h);
			double e = def_From / def_Nexthop;

			if (from_nrofCopies > 0 && h_nrofCopies != null) {
				TokenReCal(id, from, h);
			} else if ((h_nrofCopies == null) || (h_nrofCopies.intValue() == 0)) {
				if ((from.toString().contains("b")) && (from.getPath().getNextWayList().contains(m.getTo().getLocation()))) {
					if (e < 1 && h.toString().contains("b") && (h.getPath().getNextWayList().contains(m.getTo().getLocation()))) {
						m.updateProperty(MSG_COUNT_PROP, 1);
						return m;
					} else if (e > 1 && h.toString().contains("b") && (h.getPath().getNextWayList().contains(m.getTo().getLocation()))) {
						from_nrofCopies = from_nrofCopies - 1;
						m.updateProperty(MSG_COUNT_PROP, from_nrofCopies);
						return m;
					} else if (e < 1 && (h.toString().contains("b") || h.toString().contains("c"))) {
						m.updateProperty(MSG_COUNT_PROP, 0);
						return m;
					} else if (e > 1 && (h.toString().contains("b") || h.toString().contains("c"))) {
						from_nrofCopies = from_nrofCopies - 1;
						m.updateProperty(MSG_COUNT_PROP, from_nrofCopies);
						return m;
					} else {
						m.updateProperty(MSG_COUNT_PROP, from_nrofCopies);
						return m;
					}
				} else if (from.toString().contains("b") && (!from.getPath().getNextWayList().contains(m.getTo().getLocation()))) {
					if (e < 1 && h.toString().contains("b") && (h.getPath().getNextWayList().contains(m.getTo().getLocation()))) {
						m.updateProperty(MSG_COUNT_PROP, 1);
						return m;
					} else if (e > 1 && h.toString().contains("b") && (h.getPath().getNextWayList().contains(m.getTo().getLocation()))) {
						m.updateProperty(MSG_COUNT_PROP, from_nrofCopies);
						return m;
					} else if (e < 1 && (h.toString().contains("b") || h.toString().contains("c"))) {
						m.updateProperty(MSG_COUNT_PROP, from_nrofCopies);
						return m;
					} else {
						m.updateProperty(MSG_COUNT_PROP, from_nrofCopies);
						return m;
					}

				}
			}
		}
		/*
		 * nrofCopies = (int) Math.ceil(nrofCopies / 2.0);
		 * 
		 * m.updateProperty(MSG_COUNT_PROP, nrofCopies);
		 */
		return m;
	}

	@Override
	protected void transferDone(Connection con) {
		Integer nrofCopies;
		String msgId = con.getMessage().getId();
		/* get this router's copy of the message */
		Message msg = getMessage(msgId);

		if (msg == null) { // message has been dropped from the buffer after..
			return; // ..start of transfer -> no need to reduce amount of copies
		}

		if (msg.getProperty(SUMMARY_XCHG_PROP) != null) {
			deleteMessage(msgId, false);
			return;
		}

		/*
		 * reduce the amount of copies left. If the number of copies was at 1
		 * and we apparently just transferred the msg (focus phase), then we
		 * should delete it.
		 */
		nrofCopies = (Integer) msg.getProperty(MSG_COUNT_PROP);
		if (nrofCopies > 1)
			nrofCopies /= 2;
		else
			deleteMessage(msgId, false);

		msg.updateProperty(MSG_COUNT_PROP, nrofCopies);
	}

	@Override
	public void update() {
		super.update();
		if (!canStartTransfer() || isTransferring()) {
			return; // nothing to transfer or is currently transferring
		}

		/* try messages that could be delivered to final recipient */
		if (exchangeDeliverableMessages() != null) {
			return;
		}

		List<Message> spraylist = new ArrayList<Message>();
		List<Tuple<Message, Connection>> focuslist = new LinkedList<Tuple<Message, Connection>>();

		for (Message m : getMessageCollection()) {
			if (m.getProperty(SUMMARY_XCHG_PROP) != null)
				continue;

			Integer nrofCopies = (Integer) m.getProperty(MSG_COUNT_PROP);
			assert nrofCopies != null : "SnF message " + m + " didn't have " + "nrof copies property!";
			if (nrofCopies > 1) {
				spraylist.add(m);
			} else {
				/*
				 * Here we implement the single copy utility-based forwarding
				 * scheme. The utility function is the last encounter time of
				 * the msg's destination node. If our peer has a newer time
				 * (beyond the threshold), we forward the msg on to it.
				 */
				DTNHost dest = m.getTo();
				Connection toSend = null;
				double maxPeerLastSeen = 0.0; // beginning of time (simulation
												// time)

				// Get the timestamp of the last time this Host saw the
				// destination
				double thisLastSeen = getLastEncounterTimeForHost(dest);

				for (Connection c : getConnections()) {
					DTNHost peer = c.getOtherNode(getHost());
					Map<DTNHost, EncounterInfo> peerEncounters = neighborEncounters.get(peer);
					double peerLastSeen = 0.0;

					if (peerEncounters != null && peerEncounters.containsKey(dest))
						peerLastSeen = neighborEncounters.get(peer).get(dest).getLastSeenTime();

					/*
					 * We need to pick only one peer to send the copy on to; so
					 * lets find the one with the newest encounter time.
					 */

					if (peerLastSeen > maxPeerLastSeen) {
						toSend = c;
						maxPeerLastSeen = peerLastSeen;
					}

				}
				if (toSend != null && maxPeerLastSeen > thisLastSeen + transitivityTimerThreshold) {
					focuslist.add(new Tuple<Message, Connection>(m, toSend));
				}
			}
		}

		// arbitrarily favor spraying
		if (tryMessagesToConnections(spraylist, getConnections()) == null) {
			if (tryMessagesForConnected(focuslist) != null) {

			}
		}

	}

	protected double getLastEncounterTimeForHost(DTNHost host) {
		if (recentEncounters.containsKey(host))
			return recentEncounters.get(host).getLastSeenTime();
		else
			return 0.0;
	}

	/**
	 * calaulate def
	 * 
	 * @param m
	 * @param host
	 * @return def
	 */
	protected double def(Message m, DTNHost host) {
		double le, t1 = 0;

		if (host.toString().contains("b")) {// bus
			if (host.getPath().getNextWayList().contains((m.getTo().getLocation()))) {
				le = host.getLocation().distance(m.getTo().getLocation());
				t1 = (le / host.getSpeed());
				return t1;
			} else {
				le = host.getLocation().distance(m.getTo().getLocation());
				t1 = (le / host.getSpeed());
				return t1;
			}
		} else {// normal car
			if (host.getPath().getNextWaypoint().equals((m.getTo().getLocation()))) {
				le = host.getLocation().distance(m.getTo().getLocation());
				t1 = (le / host.getSpeed());
				return t1;
			}
		}
		return t1;
	}

	/**
	 * IGF token recalaulate
	 * 
	 * @param id
	 * @param from
	 * @param encounter
	 */
	protected void TokenReCal(String id, DTNHost from, DTNHost encounter) {
		Message m = super.messageTransferred(id, from);
		Integer from_nrofCopies = (Integer) m.getProperty(MSG_COUNT_PROP);
		Integer encounter_nrofCopies = (Integer) encounter.getMessageCollections().getMessage(id).getProperty(MSG_COUNT_PROP);
		double tm, tn, from_to = 0, encounter_to;
		DTNHost dest = m.getTo();

		tm = def(m, from);// my node def
		tn = def(m, encounter);// nexthop def

		from_to = from_nrofCopies;
		encounter_to = encounter_nrofCopies;
		from_to = (from_to + encounter_to) * tm / (tm + tn); // 依照def比例重新分配
		encounter_to = (from_to + encounter_to) * tn / (tm + tn);

		m.updateProperty(MSG_COUNT_PROP, from_to);
		m.updateProperty(MSG_COUNT_PROP, encounter_to);
	}

	/**
	 * Stores all necessary info about encounters made by this host to some
	 * other host. At the moment, all that's needed is the timestamp of the last
	 * time these two hosts met.
	 * 
	 * @author PJ Dillon, University of Pittsburgh
	 */
	protected class EncounterInfo {
		protected double seenAtTime;

		public EncounterInfo(double atTime) {
			this.seenAtTime = atTime;
		}

		public void updateEncounterTime(double atTime) {
			this.seenAtTime = atTime;
		}

		public double getLastSeenTime() {
			return seenAtTime;
		}

		public void updateLastSeenTime(double atTime) {
			this.seenAtTime = atTime;
		}
	}
}
